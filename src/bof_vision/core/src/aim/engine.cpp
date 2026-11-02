#include "bof_vision/aim_engine.hpp"
#include <cmath>

namespace bof {
namespace {
Eigen::Vector3d vector3(const std::vector<double>& v) {return {v[0],v[1],v[2]};}
}
bool to_reference(const AimConfig& c,const Eigen::Quaterniond& gimbal,const ArmorPoseInput& p,SpatialObservation& o) {
  const auto& q=c.extrinsics.camera_to_gimbal_q_xyzw;
  if(q.size()!=4 || c.extrinsics.camera_in_gimbal_m.size()!=3) return false;
  if(!p.optical.allFinite() || !p.rotation_vector.allFinite() || p.optical.z()<=0) return false;
  const Eigen::Quaterniond camera=Eigen::Quaterniond(q[3],q[0],q[1],q[2]).normalized();
  Eigen::Vector3d normal=Eigen::Vector3d::UnitZ();
  const double angle=p.rotation_vector.norm();
  if(angle>1e-12) normal=Eigen::AngleAxisd(angle,p.rotation_vector/angle)*normal;
  normal=gimbal*camera*normal;
  if(std::hypot(normal.x(),normal.y())<1e-6) return false;
  o={p.label,gimbal*(camera*p.optical+vector3(c.extrinsics.camera_in_gimbal_m)),std::atan2(normal.y(),normal.x())};
  return true;
}
AimEngine::AimEngine(AimConfig config)
: config_(std::move(config)), tracker_(config_), shooter_(config_.shooter),
  orientations_(ClockDomain::ros,static_cast<std::int64_t>(config_.tracking.imu_gap_s*1e9),4096) {
  if(config_.extrinsics.camera_to_gimbal_q_xyzw.size()!=4 || config_.extrinsics.camera_in_gimbal_m.size()!=3 ||
     config_.extrinsics.muzzle_in_gimbal_m.size()!=3)
    throw std::invalid_argument("AimEngine requires measured extrinsics");
  muzzle_offset_=vector3(config_.extrinsics.muzzle_in_gimbal_m);
  latency_ns_=config_.timestamp_source=="host_compensated"?static_cast<std::int64_t>(config_.camera_latency_s*1e9):0;
}
bool AimEngine::push_orientation(std::int64_t t,const Eigen::Quaterniond& q) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(t<=0 || !q.coeffs().allFinite() || std::abs(q.norm()-1)>1e-3) {++counters_.rejected_imu; return false;}
  if(last_imu_ns_ && t<last_imu_ns_) orientations_.reset();
  if(!orientations_.push({t,q})) {++counters_.rejected_imu; return false;}
  last_imu_ns_=t; latest_orientation_=q.normalized();
  return true;
}
void AimEngine::push_feedback(const GimbalFeedback& f,bool with_orientation) {
  if(with_orientation) push_orientation(f.stamp_ns,f.orientation);
  std::lock_guard<std::mutex> lock(mutex_); feedback_=f;
}
void AimEngine::set_directive(const Directive& d,std::int64_t now) {
  std::lock_guard<std::mutex> lock(mutex_);
  tracker_.set_policy(d.policy); directive_=d; directive_ns_=now;
}
SpatialState AimEngine::invalidate(const std::string& reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  pending_.clear(); tracker_.reset(); state_=SpatialState{}; state_.status=reason; last_frame_ns_=0;
  return state_;
}
std::vector<TrackUpdate> AimEngine::enqueue(PoseFrame frame,std::int64_t now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(frame.stamp_ns<=0 || (last_frame_ns_ && frame.stamp_ns<=last_frame_ns_)) {
    pending_.clear(); tracker_.reset(); state_=SpatialState{}; state_.status="pose_clock_reset";
    last_frame_ns_=frame.stamp_ns; return {{state_,{},frame.stamp_ns,std::nullopt}};
  }
  last_frame_ns_=frame.stamp_ns;
  if(pending_.size()>=32) {pending_.pop_front(); ++counters_.sync_drops;}
  // Empty frames still update the tracker so temporary loss follows the camera.
  pending_.push_back({std::move(frame),now});
  return drain_locked(now);
}
std::vector<TrackUpdate> AimEngine::drain(std::int64_t now) {
  std::lock_guard<std::mutex> lock(mutex_);
  return drain_locked(now);
}
std::vector<TrackUpdate> AimEngine::drain_locked(std::int64_t now) {
  std::vector<TrackUpdate> updates;
  while(!pending_.empty()) {
    const auto& item=pending_.front();
    const std::int64_t t=item.frame.stamp_ns-latency_ns_;
    const double age=(now-t)*1e-9, waiting=(now-item.received_ns)*1e-9;
    if(age<0 || age>config_.tracking.timeout_s || waiting>config_.tracking.sync_wait_s) {
      pending_.pop_front(); ++counters_.sync_drops;
      state_.status=age<0?"future_pose_time":"imu_sync_timeout"; continue;
    }
    const auto orientation=orientations_.at(t,ClockDomain::ros);
    if(!orientation) break;
    std::vector<SpatialObservation> observations;
    for(const auto& p:item.frame.poses) {
      SpatialObservation o;
      if(to_reference(config_,*orientation,p,o)) observations.push_back(o);
    }
    const std::int64_t frame_stamp=item.frame.stamp_ns;
    pending_.pop_front();
    state_=tracker_.update(t,observations); updates.push_back({state_,std::move(observations),frame_stamp,*orientation});
  }
  return updates;
}
std::optional<SpatialState> AimEngine::expire(std::int64_t now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!state_.stamp_ns) return std::nullopt;
  const double age=(now-state_.stamp_ns)*1e-9;
  if(age>=0 && age<=target_timeout_s(state_,config_)) return std::nullopt;
  tracker_.reset(); state_=SpatialState{}; state_.status="observation_timeout";
  return state_;
}
AimOutput AimEngine::step(std::int64_t now) {
  std::lock_guard<std::mutex> lock(mutex_);
  AimOutput out;
  out.state=state_;
  bool aim_enable=!config_.decision.require_directive, fire_enable=!config_.decision.require_directive;
  if(directive_ && (now-directive_ns_)*1e-9<config_.decision.timeout_s) {
    aim_enable=directive_->aim_enable; fire_enable=directive_->fire_enable; out.locked=directive_->policy.lock_label>=0;
  }
  // Measured speed replaces the configured one only in a plausible range (Tongji Aimer falls back below 14 m/s).
  out.bullet_speed_mps=config_.aimer.muzzle_speed_mps;
  if(config_.gimbal_link.use_bullet_speed && feedback_ && (now-feedback_->stamp_ns)*1e-9<0.5 &&
     feedback_->bullet_speed>10 && feedback_->bullet_speed<40)
    out.bullet_speed_mps=feedback_->bullet_speed;
  out.age_s=out.state.stamp_ns?(now-out.state.stamp_ns)*1e-9:0;
  if(out.state.id!=lock_track_) {lock_track_=out.state.id; lock_plate_=-1;}
  const double imu_age=(now-last_imu_ns_)*1e-9;
  const auto& q=latest_orientation_;
  if(!last_imu_ns_ || imu_age<0 || imu_age>config_.tracking.imu_gap_s) out.solution.status="imu_stale";
  else out.solution=solve_aim(out.state,config_,out.age_s,q*muzzle_offset_,out.bullet_speed_mps,&lock_plate_);
  const Eigen::Vector3d forward=q*Eigen::Vector3d::UnitX();
  const GimbalPose pose{std::atan2(forward.y(),forward.x()),std::atan2(forward.z(),std::hypot(forward.x(),forward.y()))};
  out.aiming=aim_enable && out.solution.valid;
  out.fire=shooter_.decide(out.solution,out.state.id,pose,fire_enable && out.aiming);
  if(!aim_enable && out.solution.valid) out.fire.status="aim_not_enabled";
  out.command.control=out.aiming; out.command.fire=out.fire.fire;
  if(out.aiming) {out.command.yaw=out.solution.yaw_rad; out.command.pitch=out.solution.pitch_rad;}
  output_=out;
  return out;
}
AimOutput AimEngine::output() const {std::lock_guard<std::mutex> lock(mutex_); return output_;}
AimCounters AimEngine::counters() const {std::lock_guard<std::mutex> lock(mutex_); return counters_;}
}
