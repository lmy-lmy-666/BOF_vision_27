#include "bof_vision/pipeline.hpp"
#include <chrono>

namespace bof {
PerceptionPipeline::PerceptionPipeline(const DetectorConfig& config, double timeout_s, double gate_px,const PoseConfig& pose, const AssociationConfig& association)
: detector_(make_detector(config)), tracker_(timeout_s, gate_px, association) {
  if(!pose.calibration_path.empty()) pose_solver_=std::make_unique<PoseSolver>(pose);
}
void PerceptionPipeline::reset() { tracker_.reset(); clock_.reset(); size_={}; frame_id_.clear(); }
void PerceptionPipeline::set_detection_parameters(Color color,int threshold,float confidence,float nms) {
  detector_->set_parameters(color,threshold,confidence,nms);
  reset();
}
PerceptionResult PerceptionPipeline::process(const Frame& frame) {
  auto start = std::chrono::steady_clock::now();
  if (size_ != frame.bgr.size() || frame_id_ != frame.optical_frame || clock_ != frame.clock) reset();
  size_ = frame.bgr.size(); frame_id_ = frame.optical_frame; clock_ = frame.clock;
  try {
    auto candidates = detector_->detect(frame.bgr);
    auto detected = std::chrono::steady_clock::now();
    auto track = tracker_.update(candidates, frame.stamp_ns * 1e-9);
    std::vector<PoseObservation> poses;
    if(pose_solver_) for(const auto& candidate:candidates) poses.push_back(pose_solver_->solve(candidate,frame.bgr.size()));
    auto end = std::chrono::steady_clock::now();
    return {frame.stamp_ns, frame.sequence, std::move(candidates), track,
      std::chrono::duration<double,std::milli>(detected-start).count(),
      std::chrono::duration<double,std::milli>(end-start).count(),std::move(poses)};
  } catch (...) { reset(); throw; }
}
}  // namespace bof
