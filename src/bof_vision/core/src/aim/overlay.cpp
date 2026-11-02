#include "bof_vision/aim_engine.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace bof {
namespace {
constexpr double half_width=0.0675, half_height=0.0275;   // small plate light-bar rectangle, display only
}
bool draw_aim_overlay(cv::Mat& image,const CameraCalibration& camera,const AimConfig& c,const TrackUpdate& u,const AimOutput& out) {
  const auto& s=u.state;
  std::ostringstream text;
  text << std::fixed << std::setprecision(2) << "AIM " << s.status;
  if(s.x.size()) text << " id=" << s.id << " L" << s.label << ' ' << kind_name(s.model.kind) << " w=" << s.x[7]
                      << " spin=" << spin_name(s.spin);
  cv::putText(image,text.str(),{10,50},cv::FONT_HERSHEY_SIMPLEX,0.6,{0,255,0},2);
  std::ostringstream fire;
  fire << std::fixed << std::setprecision(1) << "SOL " << out.solution.status << " plate=" << out.solution.plate
       << " fire=" << out.fire.status << " v=" << out.bullet_speed_mps;
  cv::putText(image,fire.str(),{10,75},cv::FONT_HERSHEY_SIMPLEX,0.6,{0,200,255},2);
  if(!u.orientation || !s.x.size() || c.extrinsics.camera_to_gimbal_q_xyzw.size()!=4 || c.extrinsics.camera_in_gimbal_m.size()!=3) return false;
  const auto& q=c.extrinsics.camera_to_gimbal_q_xyzw; const auto& t=c.extrinsics.camera_in_gimbal_m;
  const Eigen::Quaterniond world_camera=*u.orientation*Eigen::Quaterniond(q[3],q[0],q[1],q[2]).normalized();
  const Eigen::Vector3d camera_origin=*u.orientation*Eigen::Vector3d(t[0],t[1],t[2]);
  auto project=[&](const std::vector<Eigen::Vector3d>& world,std::vector<cv::Point2f>& pixels) {
    std::vector<cv::Point3f> points;
    for(const auto& w:world) {
      const Eigen::Vector3d p=world_camera.conjugate()*(w-camera_origin);
      if(p.z()<0.05) return false;
      points.emplace_back(float(p.x()),float(p.y()),float(p.z()));
    }
    cv::projectPoints(points,cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),camera.camera_matrix,camera.distort_coeffs,pixels);
    return true;
  };
  bool drawn=false;
  for(int i=0;i<s.model.plates;++i) {
    const auto p=predicted_plate(s,i);
    const Eigen::Vector3d centre=p.head<3>(), right(std::sin(p[3]),-std::cos(p[3]),0), up=Eigen::Vector3d::UnitZ();
    std::vector<cv::Point2f> px;
    if(!project({centre-right*half_width+up*half_height,centre+right*half_width+up*half_height,
                 centre+right*half_width-up*half_height,centre-right*half_width-up*half_height,centre},px)) continue;
    const bool unseen=s.model.kind==TargetKind::outpost && s.plate_hits[i]<c.outpost.min_plate_hits;
    const cv::Scalar colour=i==s.matched_plate?cv::Scalar(0,255,0):unseen?cv::Scalar(128,128,128):cv::Scalar(0,255,255);
    for(int k=0;k<4;++k) cv::line(image,px[k],px[(k+1)%4],colour,2);
    cv::putText(image,std::to_string(i),px[4]+cv::Point2f(-5,-12),cv::FONT_HERSHEY_SIMPLEX,0.5,colour,1);
    drawn=true;
  }
  std::vector<cv::Point2f> centre;
  if(project({{s.x[0],s.x[2],s.x[4]}},centre)) cv::circle(image,centre[0],6,{255,0,255},2);
  // Aim point at hit time, drawn in this frame's view; it leads the plate by delay + flight time.
  std::vector<cv::Point2f> aim;
  if(out.solution.valid && out.state.id==s.id && project({out.solution.point},aim)) {
    cv::drawMarker(image,aim[0],{0,0,255},cv::MARKER_CROSS,18,2);
    if(out.fire.fire) cv::circle(image,aim[0],12,{0,0,255},2);
  }
  return drawn;
}
}
