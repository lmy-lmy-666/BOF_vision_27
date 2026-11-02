#pragma once
// ROS-free auto-aim engine: camera/IMU synchronisation, tracking, fire control and the gimbal
// command. aim_node only converts messages and drives the threads. All stamps share one clock (ns).
#include "bof_vision/aim.hpp"
#include "bof_vision/gimbal_link.hpp"
#include "bof_vision/orientation_buffer.hpp"
#include "bof_vision/pose.hpp"
#include <deque>
#include <mutex>
#include <optional>

namespace bof {
struct ArmorPoseInput {
  int label=-1;
  Eigen::Vector3d optical=Eigen::Vector3d::Zero();         // camera optical position, metres
  Eigen::Vector3d rotation_vector=Eigen::Vector3d::Zero(); // plate -> optical Rodrigues vector
};
struct PoseFrame {
  std::int64_t stamp_ns=0;                // receive stamp; host_compensated subtracts camera_latency_s
  std::vector<ArmorPoseInput> poses;     // valid, non-ambiguous poses only; may be empty
};
struct Directive {
  bool aim_enable=true, fire_enable=false;
  TargetPolicy policy;
};
struct AimOutput {
  SpatialState state;
  AimSolution solution;
  FireDecision fire;
  GimbalCommand command;
  double age_s=0, bullet_speed_mps=0;
  bool aiming=false, locked=false;
};
struct AimCounters {std::uint64_t sync_drops=0, rejected_imu=0;};
// One tracker update and the reference-frame observations that produced it.
struct TrackUpdate {
  SpatialState state;
  std::vector<SpatialObservation> observations;
  std::int64_t frame_stamp_ns=0;                                   // camera frame stamp as received
  std::optional<Eigen::Quaterniond> orientation;                   // gimbal attitude at exposure
};
// Camera optical pose -> reference observation, given the gimbal orientation at exposure.
// Returns false for invalid input or a plate normal parallel to gravity.
bool to_reference(const AimConfig& config, const Eigen::Quaterniond& gimbal, const ArmorPoseInput& pose,
                  SpatialObservation& observation);
// Debug overlay: target plates, rotation centre and aim point projected into the frame of `update`
// using its exposure attitude. Matched plate green, other usable plates yellow, unseen outpost plates
// grey, aim point red. Returns false when nothing could be projected.
bool draw_aim_overlay(cv::Mat& bgr, const CameraCalibration& camera, const AimConfig& config,
                      const TrackUpdate& update, const AimOutput& output);
class AimEngine {
public:
  explicit AimEngine(AimConfig config);
  // Thread-safe inputs.
  bool push_orientation(std::int64_t stamp_ns, const Eigen::Quaterniond& orientation);
  // with_orientation=false when the attitude arrives separately (replay of gimbal/imu + gimbal/state).
  void push_feedback(const GimbalFeedback& feedback, bool with_orientation=true);
  void set_directive(const Directive& directive, std::int64_t now_ns);
  // Queue a camera frame and process every frame whose orientation is available.
  // Returns the tracker states produced, in order, for publishing.
  std::vector<TrackUpdate> enqueue(PoseFrame frame, std::int64_t now_ns);
  std::vector<TrackUpdate> drain(std::int64_t now_ns);
  // Drops the target when its observation is older than the target timeout.
  std::optional<SpatialState> expire(std::int64_t now_ns);
  SpatialState invalidate(const std::string& reason);
  // Command-loop body: predict to now, choose plate, decide fire and build the gimbal command.
  AimOutput step(std::int64_t now_ns);
  AimOutput output() const;
  AimCounters counters() const;
  const AimConfig& config() const {return config_;}
private:
  std::vector<TrackUpdate> drain_locked(std::int64_t now_ns);
  struct Pending {PoseFrame frame; std::int64_t received_ns;};
  AimConfig config_;
  SpatialTracker tracker_;
  Shooter shooter_;
  OrientationBuffer orientations_;
  Eigen::Quaterniond latest_orientation_=Eigen::Quaterniond::Identity();
  Eigen::Vector3d muzzle_offset_;
  std::int64_t latency_ns_=0, last_imu_ns_=0, last_frame_ns_=0, directive_ns_=0;
  mutable std::mutex mutex_;
  std::deque<Pending> pending_;
  SpatialState state_;
  std::optional<GimbalFeedback> feedback_;
  std::optional<Directive> directive_;
  AimOutput output_;
  AimCounters counters_;
  std::uint64_t lock_track_=0;
  int lock_plate_=-1;
};
}
