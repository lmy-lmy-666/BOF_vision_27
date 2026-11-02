#include "bof_vision/aim.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <set>
#include <stdexcept>

namespace bof {
namespace {
void check_keys(const YAML::Node& n, const std::set<std::string>& allowed, const std::string& where) {
  if(!n.IsMap()) throw std::invalid_argument(where+" must be a map");
  std::set<std::string> seen;
  for(auto item:n) {
    auto k=item.first.as<std::string>();
    if(!allowed.count(k) || !seen.insert(k).second) throw std::invalid_argument("unknown/duplicate field: "+where+"."+k);
  }
}
#define TOP_FIELDS(F) F(enabled) F(origin) F(reference_frame) F(camera_frame) F(imu_frame) \
  F(timestamp_source) F(camera_latency_s) F(command_rate_hz) F(publish_rate_hz) F(publish_gimbal_io) F(debug_image_rate_hz)
#define EXTRINSICS_FIELDS(F) F(camera_to_gimbal_q_xyzw) F(camera_in_gimbal_m) F(muzzle_in_gimbal_m)
#define GIMBAL_LINK_FIELDS(F) F(imu_source) F(protocol) F(serial_port) F(serial_baud) F(pitch_up_positive) F(use_bullet_speed) F(use_enemy_color)
#define SELECTION_FIELDS(F) F(priority) F(switch_frames)
#define TRACKING_FIELDS(F) F(timeout_s) F(imu_gap_s) F(sync_wait_s) F(position_gate_m) F(yaw_gate_rad) \
  F(ambiguity_margin) F(acceleration_variance) F(angular_acceleration_variance) F(position_variance) \
  F(yaw_variance) F(shape_variance) F(initial_variance) F(min_radius_m) F(max_radius_m) \
  F(max_height_difference_m) F(confirm_frames)
#define OUTPOST_FIELDS(F) F(spin) F(speed_radps) F(timeout_s) F(vote_samples) F(vote_interval) \
  F(min_plate_hits) F(vote_stable_radps) F(vote_jump_rad) F(coming_angle_deg) F(leaving_angle_deg)
#define AIMER_FIELDS(F) F(muzzle_speed_mps) F(gravity_mps2) F(control_delay_s) F(high_speed_threshold_radps) \
  F(high_speed_control_delay_s) F(yaw_offset_deg) F(pitch_offset_deg) F(max_prediction_s) \
  F(spin_threshold_radps) F(max_facing_angle_rad) F(coming_angle_deg) F(leaving_angle_deg)
#define SHOOTER_FIELDS(F) F(auto_fire) F(half_width_m) F(half_height_m) F(min_tolerance_rad) F(max_tolerance_rad)
#define DECISION_FIELDS(F) F(enabled) F(require_directive) F(timeout_s)
// Each group is a YAML sub-map of aim with the same name as the AimConfig member.
#define GROUPS(G) G(extrinsics,EXTRINSICS_FIELDS) G(gimbal_link,GIMBAL_LINK_FIELDS) G(selection,SELECTION_FIELDS) \
  G(tracking,TRACKING_FIELDS) G(outpost,OUTPOST_FIELDS) G(aimer,AIMER_FIELDS) G(shooter,SHOOTER_FIELDS) \
  G(decision,DECISION_FIELDS)
#define KEY(k) #k,
TargetKind parse_kind(const std::string& v) {
  if(v=="vehicle") return TargetKind::vehicle;
  if(v=="outpost") return TargetKind::outpost;
  throw std::invalid_argument("aim.targets kind must be vehicle or outpost");
}
}
const char* kind_name(TargetKind k) {return k==TargetKind::outpost?"outpost":"vehicle";}
void load_aim_config(const YAML::Node& n, AimConfig& c) {
#define GROUP_KEY(g,FIELDS) #g,
  check_keys(n, {TOP_FIELDS(KEY) GROUPS(GROUP_KEY) "targets"}, "aim");
#define READ(k) if(n[#k]) c.k=n[#k].as<decltype(c.k)>();
  TOP_FIELDS(READ)
#define GROUP_READ(g,FIELDS) if(n[#g]) { const auto m=n[#g]; auto& s=c.g; \
    check_keys(m,{FIELDS(KEY)},"aim." #g); FIELDS(SUB_READ) }
#define SUB_READ(k) if(m[#k]) s.k=m[#k].as<decltype(s.k)>();
  GROUPS(GROUP_READ)
  if(n["targets"]) {
    if(!n["targets"].IsMap()) throw std::invalid_argument("aim.targets must map numeric labels to models");
    c.targets.clear();
    for(auto item:n["targets"]) {
      int label=item.first.as<int>(); const auto m=item.second;
      check_keys(m,{"kind","plates","radius_m"},"aim.targets");
      TargetModel model;
      model.kind=parse_kind(m["kind"].as<std::string>()); model.plates=m["plates"].as<int>(); model.radius_m=m["radius_m"].as<double>();
      if(!c.targets.emplace(label,model).second) throw std::invalid_argument("duplicate aim target label");
    }
  }
}
YAML::Node aim_config_yaml(const AimConfig& c) {
  YAML::Node n(YAML::NodeType::Map);
#define WRITE(k) n[#k]=c.k;
  TOP_FIELDS(WRITE)
#define GROUP_WRITE(g,FIELDS) { YAML::Node m(YAML::NodeType::Map); const auto& s=c.g; FIELDS(SUB_WRITE) n[#g]=m; }
#define SUB_WRITE(k) m[#k]=s.k;
  GROUPS(GROUP_WRITE)
  n["targets"]=YAML::Node(YAML::NodeType::Map);
  for(const auto& [label,m]:c.targets) {
    n["targets"][label]["kind"]=kind_name(m.kind); n["targets"][label]["plates"]=m.plates; n["targets"][label]["radius_m"]=m.radius_m;
  }
  return n;
}
void validate_aim_config(const AimConfig& c) {
  auto positive=[](double v){return std::isfinite(v)&&v>0;};
  auto non_negative=[](double v){return std::isfinite(v)&&v>=0;};
  const auto& t=c.tracking; const auto& o=c.outpost; const auto& a=c.aimer; const auto& s=c.shooter;
  for(double v:{t.timeout_s,t.imu_gap_s,t.sync_wait_s,t.position_gate_m,t.yaw_gate_rad,
      t.acceleration_variance,t.angular_acceleration_variance,t.position_variance,t.yaw_variance,t.shape_variance,
      t.initial_variance,t.min_radius_m,t.max_radius_m,t.max_height_difference_m,
      o.timeout_s,o.vote_stable_radps,o.vote_jump_rad,o.coming_angle_deg,o.leaving_angle_deg,
      a.gravity_mps2,a.max_prediction_s,a.spin_threshold_radps,a.max_facing_angle_rad,a.coming_angle_deg,a.leaving_angle_deg,
      s.half_width_m,s.half_height_m,s.min_tolerance_rad,s.max_tolerance_rad,
      c.decision.timeout_s,c.command_rate_hz,c.publish_rate_hz})
    if(!positive(v)) throw std::invalid_argument("aim time/noise/geometry/gate/rate values must be positive and finite");
  for(double v:{a.muzzle_speed_mps,a.control_delay_s,a.high_speed_threshold_radps,a.high_speed_control_delay_s,
      c.camera_latency_s,o.speed_radps,t.ambiguity_margin})
    if(!non_negative(v)) throw std::invalid_argument("aim speeds/delays/margins must be finite and non-negative");
  if(!std::isfinite(a.yaw_offset_deg) || !std::isfinite(a.pitch_offset_deg) || std::abs(a.yaw_offset_deg)>10 || std::abs(a.pitch_offset_deg)>10)
    throw std::invalid_argument("aim.aimer offsets must be finite and within 10 degrees");
  if(t.confirm_frames<2 || c.selection.switch_frames<1 || o.vote_samples<o.vote_interval+5 || o.vote_interval<1 ||
     o.min_plate_hits<1 || t.min_radius_m>=t.max_radius_m || t.yaw_gate_rad>3.141592653589793 ||
     a.max_facing_angle_rad>1.570796326794897 || s.min_tolerance_rad>s.max_tolerance_rad ||
     a.leaving_angle_deg>a.coming_angle_deg || o.leaving_angle_deg>o.coming_angle_deg ||
     a.coming_angle_deg>90 || o.coming_angle_deg>90 || c.command_rate_hz>2000 || c.publish_rate_hz>c.command_rate_hz)
    throw std::invalid_argument("invalid aim bounds");
  if(c.timestamp_source!="host_receive" && c.timestamp_source!="host_compensated" &&
     c.timestamp_source!="exposure_synchronized" && c.timestamp_source!="simulation")
    throw std::invalid_argument("aim.timestamp_source must be host_receive/host_compensated/exposure_synchronized/simulation");
  if(c.origin!="stationary" && c.origin!="gimbal_centered") throw std::invalid_argument("aim.origin must be stationary or gimbal_centered");
  if(c.gimbal_link.imu_source!="topic" && c.gimbal_link.imu_source!="serial")
    throw std::invalid_argument("aim.gimbal_link.imu_source must be topic or serial");
  if(c.gimbal_link.protocol!="cboard" && c.gimbal_link.protocol!="sp")
    throw std::invalid_argument("aim.gimbal_link.protocol must be cboard or sp");
  if(o.spin!="auto" && o.spin!="stopped") throw std::invalid_argument("aim.outpost.spin must be auto or stopped");
  std::set<int> seen;
  for(int label:c.selection.priority)
    if(label<0 || label>7 || !seen.insert(label).second) throw std::invalid_argument("aim.selection.priority needs unique labels 0..7");
  auto vector=[](const auto& v,std::size_t n){if(v.size()!=n) return false; for(auto x:v) if(!std::isfinite(x)) return false; return true;};
  const auto& e=c.extrinsics;
  if(!e.camera_to_gimbal_q_xyzw.empty()) {
    if(!vector(e.camera_to_gimbal_q_xyzw,4)) throw std::invalid_argument("aim camera quaternion requires 4 finite xyzw values");
    auto q=e.camera_to_gimbal_q_xyzw;
    if(std::abs(Eigen::Quaterniond(q[3],q[0],q[1],q[2]).norm()-1)>1e-3) throw std::invalid_argument("aim camera quaternion must be unit length");
  }
  for(const auto* v:{&e.camera_in_gimbal_m,&e.muzzle_in_gimbal_m})
    if(!v->empty()&&!vector(*v,3)) throw std::invalid_argument("aim translations require 3 finite metre values");
  for(auto [label,m]:c.targets) {
    if(label<0 || label>7 || !positive(m.radius_m) || m.radius_m<t.min_radius_m || m.radius_m>t.max_radius_m ||
       !((m.kind==TargetKind::vehicle&&(m.plates==2||m.plates==4)) || (m.kind==TargetKind::outpost&&m.plates==3)))
      throw std::invalid_argument("aim targets require vehicle(2/4) or outpost(3), label 0..7 and explicit radius");
  }
  if(!c.decision.enabled && c.decision.require_directive)
    throw std::invalid_argument("aim.decision.require_directive needs decision.enabled=true");
  if(!std::isfinite(c.debug_image_rate_hz) || c.debug_image_rate_hz<0 || c.debug_image_rate_hz>60)
    throw std::invalid_argument("aim.debug_image_rate_hz must be in [0,60]");
  if(c.timestamp_source=="host_compensated" && !positive(c.camera_latency_s))
    throw std::invalid_argument("aim.host_compensated requires measured camera_latency_s > 0");
  if(c.gimbal_link.imu_source=="serial" && (c.gimbal_link.serial_port.empty() || c.gimbal_link.serial_baud<=0))
    throw std::invalid_argument("aim.gimbal_link.imu_source=serial requires serial_port and serial_baud");
  if(c.enabled && (c.reference_frame.empty() || c.camera_frame.empty() || c.imu_frame.empty() ||
     !vector(e.camera_to_gimbal_q_xyzw,4) || !vector(e.camera_in_gimbal_m,3) || !vector(e.muzzle_in_gimbal_m,3) ||
     !positive(a.muzzle_speed_mps) || c.targets.empty()))
    throw std::invalid_argument("aim.enabled requires measured extrinsics, aimer.muzzle_speed_mps and target models");
}
}
