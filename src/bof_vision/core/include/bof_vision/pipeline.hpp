#pragma once
#include "bof_vision/detector_factory.hpp"
#include "bof_vision/pose.hpp"
#include <string>

namespace bof {
enum class ClockDomain { nominal_video, ros, host_receive, exposure_synchronized };
struct Frame {
  cv::Mat bgr;  // Borrowed for synchronous process(); caller keeps pixels alive and immutable.
  std::int64_t stamp_ns = 0;
  std::uint64_t sequence = 0;
  std::string optical_frame = "camera_optical_frame";
  ClockDomain clock = ClockDomain::nominal_video;
};
struct PerceptionResult {
  std::int64_t stamp_ns;
  std::uint64_t sequence;
  std::vector<Candidate> candidates;
  PixelTrack track;
  double detect_ms;
  double process_ms;
  std::vector<PoseObservation> poses;  // Empty when disabled; otherwise aligns with candidates.
};
// Single-consumer synchronous pipeline. ROS transport and GUI are outside this class.
class PerceptionPipeline {
public:
  explicit PerceptionPipeline(const DetectorConfig& config, double timeout_s=0.2, double gate_px=100,
                              const PoseConfig& pose={}, const AssociationConfig& association={});
  PerceptionResult process(const Frame& frame);
  void reset();
  // Single-consumer update; does not rebuild or recompile the detector.
  void set_detection_parameters(Color color,int threshold,float confidence,float nms);
private:
  std::unique_ptr<Detector> detector_;
  PixelTracker tracker_;
  std::unique_ptr<PoseSolver> pose_solver_;
  cv::Size size_;
  std::string frame_id_;
  std::optional<ClockDomain> clock_;
};
}  // namespace bof
