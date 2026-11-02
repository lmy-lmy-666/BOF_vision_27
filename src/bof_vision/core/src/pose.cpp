// Planar four-corner PnP/reprojection workflow adapted from
// TongjiSuperPower/sp_vision_25 tasks/auto_aim/solver.cpp (MIT).
// Copyright (c) 2025 TongjiSuperPower. See third_party/licenses/TongjiSuperPower-MIT.txt.
#include "bof_vision/pose.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace bof {
CameraCalibration load_calibration(const std::string& path) {
  const auto yaml=YAML::LoadFile(path);
  CameraCalibration calibration;
  calibration.image_size={yaml["image_width"].as<int>(),yaml["image_height"].as<int>()};
  if(calibration.image_size.width<=0 || calibration.image_size.height<=0)
    throw std::invalid_argument("calibration image_width/image_height must be positive");
  if(yaml["distortion_model"] && yaml["distortion_model"].as<std::string>()!="opencv_pinhole")
    throw std::invalid_argument("only opencv_pinhole calibration is supported (no fisheye)");
  const auto matrix=yaml["camera_matrix"].as<std::vector<double>>();
  const auto distortion=yaml["distort_coeffs"].as<std::vector<double>>();
  if(matrix.size()!=9 || (distortion.size()!=4 && distortion.size()!=5 && distortion.size()!=8))
    throw std::invalid_argument("camera_matrix requires 9 values; distort_coeffs requires 4/5/8");
  for(double value:matrix) if(!std::isfinite(value)) throw std::invalid_argument("non-finite camera matrix");
  for(double value:distortion) if(!std::isfinite(value)) throw std::invalid_argument("non-finite distortion");
  if(matrix[0]<=0 || matrix[4]<=0 || std::abs(matrix[1])>1e-9 || std::abs(matrix[3])>1e-9 ||
     std::abs(matrix[6])>1e-9 || std::abs(matrix[7])>1e-9 || std::abs(matrix[8]-1)>1e-9)
    throw std::invalid_argument("invalid pinhole camera matrix");
  calibration.camera_matrix=cv::Mat(3,3,CV_64F,const_cast<double*>(matrix.data())).clone();
  calibration.distort_coeffs=cv::Mat(1,static_cast<int>(distortion.size()),CV_64F,const_cast<double*>(distortion.data())).clone();
  return calibration;
}
PoseSolver::PoseSolver(const PoseConfig& config) : config_(config),calibration_(load_calibration(config.calibration_path)) {
  if(!std::isfinite(config.max_reprojection_error_px) || config.max_reprojection_error_px<=0)
    throw std::invalid_argument("PnP reprojection limit must be finite and positive");
  if(!config.geometry_path.empty()) {
    if(config.plate_width_m!=0 || config.plate_height_m!=0 || config.label!=-2)
      throw std::invalid_argument("choose geometry_path OR fixed plate dimensions/label");
    geometry_=std::make_unique<GeometryResolver>(config.geometry_path);
    return;
  }
  if(!std::isfinite(config.plate_width_m) || !std::isfinite(config.plate_height_m) ||
     config.plate_width_m<=0 || config.plate_height_m<=0 || config.label < -1 || config.label > 7 ||
     !std::isfinite(config.max_reprojection_error_px) || config.max_reprojection_error_px<=0)
    throw std::invalid_argument("PnP requires explicit positive plate dimensions, selected label and error limit");
  const double w=config.plate_width_m/2,h=config.plate_height_m/2;
  // TL/TR/BR/BL. Our plate frame differs from Tongji's x-normal/y-left/z-up frame.
  points_={{-w,-h,0},{w,-h,0},{w,h,0},{-w,h,0}};
}
PoseObservation PoseSolver::solve(const Candidate& candidate,cv::Size image_size) const {
  PoseObservation result;
  if(image_size!=calibration_.image_size) { result.status="resolution_mismatch"; return result; }
  if(!geometry_ && static_cast<int>(candidate.label)!=config_.label) { result.status="label_filtered"; return result; }
  const std::vector<cv::Point2f> image_points(candidate.corners.begin(),candidate.corners.end());
  for(const auto& point:image_points)
    if(!std::isfinite(point.x) || !std::isfinite(point.y)) { result.status="invalid_corners"; return result; }
  if(!cv::isContourConvex(image_points) || std::abs(cv::contourArea(image_points))<1) {
    result.status="degenerate_corners"; return result;
  }
  std::vector<cv::Point3d> points=points_;
  if(geometry_) {
    const auto selected=geometry_->choose(candidate);
    result.geometry=plate_size_name(selected.size); result.geometry_source=selected.source;
    result.width_m=selected.width_m; result.height_m=selected.height_m; result.image_ratio=selected.ratio;
    if(!selected.valid) { result.status=selected.status; return result; }
    const double w=selected.width_m/2,h=selected.height_m/2;
    points={{-w,-h,0},{w,-h,0},{w,h,0},{-w,h,0}};
  } else {
    result.geometry="fixed"; result.geometry_source="explicit";
    result.width_m=config_.plate_width_m; result.height_m=config_.plate_height_m;
  }
  try {
    std::vector<cv::Mat> rotations,translations;
    cv::solvePnPGeneric(points,image_points,calibration_.camera_matrix,calibration_.distort_coeffs,
                        rotations,translations,false,cv::SOLVEPNP_IPPE);
    struct Solution { cv::Vec3d r,t; double error; };
    std::vector<Solution> solutions;
    for(std::size_t i=0;i<rotations.size();++i) {
      const auto& r=rotations[i]; const auto& t=translations[i];
      if(!cv::checkRange(r) || !cv::checkRange(t)) continue;
      const cv::Vec3d rv(r.at<double>(0),r.at<double>(1),r.at<double>(2));
      const cv::Vec3d tv(t.at<double>(0),t.at<double>(1),t.at<double>(2));
      cv::Matx33d rotation;
      cv::Rodrigues(rv,rotation);
      bool positive=true;
      for(const auto& p:points) if((rotation*cv::Vec3d(p.x,p.y,p.z)+tv)[2]<=0) positive=false;
      if(!positive) continue;
      std::vector<cv::Point2d> projected;
      cv::projectPoints(points,rv,tv,calibration_.camera_matrix,calibration_.distort_coeffs,projected);
      double squared=0;
      for(std::size_t j=0;j<projected.size();++j) {
        const auto delta=projected[j]-cv::Point2d(image_points[j]); squared+=delta.dot(delta);
      }
      const double error=std::sqrt(squared/projected.size());
      if(std::isfinite(error)) solutions.push_back({rv,tv,error});
    }
    if(solutions.empty()) { result.status="no_positive_solution"; return result; }
    std::sort(solutions.begin(),solutions.end(),[](const auto& a,const auto& b){return a.error<b.error;});
    result.position_m=solutions.front().t; result.rotation_vector=solutions.front().r;
    result.distance_m=cv::norm(result.position_m); result.reprojection_error_px=solutions.front().error;
    if(solutions.size()>1) {
      result.alternative_error_px=solutions[1].error;
      // Diagnostic only: near-equal reprojection does not establish unique orientation.
      result.ambiguous=solutions[1].error-solutions[0].error<0.5;
    }
    result.valid=result.reprojection_error_px<=config_.max_reprojection_error_px;
    result.status=result.valid ? "valid" : "reprojection_error";
  } catch(const cv::Exception&) { result.status="pnp_failed"; }
  return result;
}
}  // namespace bof
