#pragma once
#include "bof_vision/core.hpp"
#include "bof_vision/geometry.hpp"
#include <memory>
#include <string>

namespace bof {
struct PoseConfig {
  std::string calibration_path;
  std::string geometry_path;  // Multi-label small/large policies; alternative to fixed width/height/label.
  double plate_width_m = 0;
  double plate_height_m = 0;
  int label = -2;  // Explicit selection required; -1 means geometric/unknown candidates.
  double max_reprojection_error_px = 4;
};
struct CameraCalibration {
  cv::Size image_size;
  cv::Mat camera_matrix;
  cv::Mat distort_coeffs;
};
CameraCalibration load_calibration(const std::string& path);
struct PoseObservation {
  bool valid = false;
  std::string status;
  cv::Vec3d position_m{};  // Camera optical: x right, y down, z forward.
  cv::Vec3d rotation_vector{};  // Plate local x right/y down/z normal -> camera optical.
  double distance_m = 0;
  double reprojection_error_px = -1;
  double alternative_error_px = -1;
  bool ambiguous = false;
  std::string geometry="unknown",geometry_source="none";
  double width_m=0,height_m=0,image_ratio=0;
};
// Tongji's planar IPPE workflow, adapted to explicit geometry and our corner order.
// No world/gimbal transformation, time prediction or aiming output.
class PoseSolver {
public:
  explicit PoseSolver(const PoseConfig& config);
  PoseObservation solve(const Candidate& candidate,cv::Size image_size) const;
  const CameraCalibration& calibration() const { return calibration_; }
private:
  PoseConfig config_;
  CameraCalibration calibration_;
  std::vector<cv::Point3d> points_;
  std::unique_ptr<GeometryResolver> geometry_;
};
}  // namespace bof
