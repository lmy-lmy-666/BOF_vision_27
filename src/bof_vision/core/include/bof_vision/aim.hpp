#pragma once
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include <array>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace bof {
enum class TargetKind {vehicle, outpost};
const char* kind_name(TargetKind kind);
struct TargetModel {
  TargetKind kind=TargetKind::vehicle;
  int plates=4;
  double radius_m=0;
};
// Measured transforms: camera optical -> IMU/gimbal; positions in IMU/gimbal. Barrel axis is IMU +x.
struct ExtrinsicsConfig {
  std::vector<double> camera_to_gimbal_q_xyzw, camera_in_gimbal_m, muzzle_in_gimbal_m;
};
struct GimbalLinkConfig {
  std::string imu_source="topic";   // topic (/gimbal/imu) or serial (gimbal link quaternion)
  std::string protocol="cboard";    // cboard: team firmware sentry_gimbol_new; sp: Tongji sp_vision_25
  std::string serial_port;
  int serial_baud=115200;
  bool pitch_up_positive=false;     // sp protocol only; cboard conventions are fixed by the firmware
  // The firmware currently sends a constant 22 m/s and bullet_count=1; enable once it reports real values.
  bool use_bullet_speed=false;
  // Publish the firmware enemy colour on gimbal/enemy_color for pipeline_node. Off by default: the
  // firmware derives it from robot_id, which is 0 (reported as blue enemy) until the referee connects.
  bool use_enemy_color=false;
};
struct SelectionConfig {
  // Labels earlier in the list win; a better target must persist switch_frames.
  std::vector<int> priority{1,3,4,5,0,2,6,7};
  int switch_frames=5;
};
struct TrackingConfig {
  double timeout_s=0.2, imu_gap_s=0.05, sync_wait_s=0.08;
  double position_gate_m=0.5, yaw_gate_rad=0.8, ambiguity_margin=0.15;
  double acceleration_variance=20, angular_acceleration_variance=50;
  double position_variance=0.01, yaw_variance=0.04, shape_variance=0.0001;
  double initial_variance=1, min_radius_m=0.01, max_radius_m=1, max_height_difference_m=0.5;
  int confirm_frames=5;
};
// RPS direction vote. auto votes; stopped pins omega=0.
struct OutpostConfig {
  std::string spin="auto";
  double speed_radps=2.513274, timeout_s=1.0;
  int vote_samples=30, vote_interval=3, min_plate_hits=3;
  double vote_stable_radps=0.6, vote_jump_rad=0.4;
  double coming_angle_deg=70, leaving_angle_deg=30;
};
// Plate choice and trajectory (Tongji Aimer).
struct AimerConfig {
  double muzzle_speed_mps=0, gravity_mps2=9.80665, control_delay_s=0;
  double high_speed_threshold_radps=0, high_speed_control_delay_s=0;
  double yaw_offset_deg=0, pitch_offset_deg=0, max_prediction_s=1.5;
  double spin_threshold_radps=2.0, max_facing_angle_rad=1.0, coming_angle_deg=60, leaving_angle_deg=20;
};
// Fire decision (Tongji Shooter, tolerance from plate size as in RPS isGimbalAngleOk).
struct ShooterConfig {
  bool auto_fire=false;
  double half_width_m=0.06, half_height_m=0.05, min_tolerance_rad=0.005, max_tolerance_rad=0.087;
};
struct DecisionConfig {
  // false: auto aim runs standalone; aim_node neither subscribes to /sentry/aim_directive nor
  // publishes /auto_aim/target, so navigation/decision and vision cannot affect each other.
  bool enabled=false;
  bool require_directive=false;     // true: no gimbal control or fire without a fresh directive
  double timeout_s=0.5;
};
struct AimConfig {
  bool enabled=false;
  // stationary: origin fixed at a static gimbal centre (bench).
  // gimbal_centered: Tongji sp_vision_25 convention, origin moves with the gimbal; chassis translation
  // is absorbed by the target velocity states (no odometry compensation).
  std::string origin="stationary";
  std::string reference_frame="aim_reference", camera_frame="camera_optical_frame", imu_frame="gimbal";
  // host_compensated: host receive stamp minus measured camera_latency_s is used as exposure time.
  std::string timestamp_source="host_receive";
  double camera_latency_s=0, command_rate_hz=500, publish_rate_hz=50;
  bool publish_gimbal_io=true;   // gimbal/imu, gimbal/state, gimbal/command topics for rosbag recording
  double debug_image_rate_hz=10; // aim/debug_image overlay on debug/image; 0 disables (matches)
  ExtrinsicsConfig extrinsics;
  GimbalLinkConfig gimbal_link;
  std::map<int,TargetModel> targets;
  SelectionConfig selection;
  TrackingConfig tracking;
  OutpostConfig outpost;
  AimerConfig aimer;
  ShooterConfig shooter;
  DecisionConfig decision;
};
void load_aim_config(const YAML::Node& node, AimConfig& config);
void validate_aim_config(const AimConfig& config);
YAML::Node aim_config_yaml(const AimConfig& config);
struct SpatialObservation {
  int label=-1;
  Eigen::Vector3d position=Eigen::Vector3d::Zero();
  double inward_yaw=0;
};
enum class Spin {unknown, counter_clockwise, clockwise, stopped};
const char* spin_name(Spin spin);
struct SpatialState {
  bool valid=false;
  std::string status="uninitialized";
  std::uint64_t id=0;
  int label=-1, matched_plate=-1;
  TargetModel model;
  // Vehicle: x,vx,y,vy,z,vz,yaw,omega,r,dr,dz (Tongji).
  // Outpost: x,vx,y,vy,z,vz,yaw,omega,dz1,dz2 (RPS, fixed radius).
  Eigen::VectorXd x;
  std::int64_t stamp_ns=0;
  Spin spin=Spin::unknown;                 // outpost only
  std::array<int,4> plate_hits{0,0,0,0};   // updates per plate since initialisation
};
// Decision input applied to target choice. Labels follow ArmorCandidate.
struct TargetPolicy {
  std::set<int> excluded;      // invincible / not to be attacked
  int lock_label=-1;           // only this label when >=0
  int priority_label=-1;       // ranked first when >=0
  bool outpost_stopped=false;  // overrides the vote
};
// RPS OutpostModel::calDirection: compare orientation n samples apart and vote; the stationary
// threshold is an angular rate so the result does not depend on the camera frame rate.
Spin vote_direction(const std::vector<double>& yaws, const std::vector<double>& times_s, int interval,
                    double stable_radps, double jump_rad);
class SpatialTracker {
public:
  explicit SpatialTracker(AimConfig config);
  SpatialState update(std::int64_t stamp_ns, const std::vector<SpatialObservation>& observations);
  void set_policy(TargetPolicy policy);
  void reset();
private:
  int rank(int label) const;
  void initialize(std::int64_t t, const SpatialObservation& o);
  void pin_spin();
  void vote(double yaw);
  AimConfig config_;
  TargetPolicy policy_;
  SpatialState state_;
  Eigen::MatrixXd covariance_;
  std::int64_t last_seen_=0;
  int hits_=0, better_frames_=0, disagreements_=0;
  bool confirmed_=false;
  Spin pending_=Spin::unknown;
  std::vector<double> vote_yaws_, vote_times_;
  std::uint64_t next_id_=0;
};
double target_timeout_s(const SpatialState& state, const AimConfig& config);
Eigen::Vector4d predicted_plate(const SpatialState& state, int index, double dt=0);
struct AimSolution {
  bool valid=false;
  std::string status="uninitialized";
  int plate=-1;
  double yaw_rad=0, pitch_rad=0, flight_time_s=0, distance_m=0, facing_rad=0;
  bool in_window=true;   // false: pre-aiming the next plate of a spinning target, not fireable
  Eigen::Vector3d point=Eigen::Vector3d::Zero();
};
// Vacuum low-arc trajectory adapted from Tongji tools/trajectory.cpp (MIT); plate choice from
// Tongji Aimer::choose_aim_point. lock_plate keeps the slow-target lock between calls (may be null).
// muzzle_speed_mps overrides aimer.muzzle_speed_mps (measured speed). Angles are absolute
// reference-frame azimuth/elevation including configured offsets.
AimSolution solve_aim(const SpatialState& state, const AimConfig& config, double observation_age_s,
                      const Eigen::Vector3d& muzzle, double muzzle_speed_mps, int* lock_plate=nullptr);
struct GimbalPose {double yaw_rad=0, pitch_rad=0;};
struct FireDecision {bool fire=false; std::string status="no_solution";};
// Tongji Shooter: command must be steady and the gimbal must already point within the plate.
class Shooter {
public:
  explicit Shooter(ShooterConfig config):config_(config) {}
  FireDecision decide(const AimSolution& solution, std::uint64_t track_id, const GimbalPose& gimbal, bool enabled);
private:
  ShooterConfig config_;
  bool has_last_=false;
  std::uint64_t last_track_=0;
  int last_plate_=-1;
  double last_yaw_=0;
};
}
