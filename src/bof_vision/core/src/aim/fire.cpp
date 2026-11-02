// Plate choice, trajectory and fire decision adapted from Tongji sp_vision_25 Aimer/Shooter and
// tools/trajectory.cpp, with the outpost window/height idea from RPS AngleSolver (MIT).
// See third_party/NOTICE.md and licenses/{TongjiSuperPower,RPS}-MIT.txt.
#include "bof_vision/aim.hpp"
#include <algorithm>
#include <cmath>

namespace bof {
namespace {
constexpr double pi=3.14159265358979323846;
constexpr double degree=pi/180;
double wrap(double x) { return std::remainder(x,2*pi); }
// Angle between a plate normal and the line from the muzzle to the target centre; 0 faces the muzzle.
double facing(const SpatialState& s,int index,double dt,const Eigen::Vector3d& muzzle) {
  const auto& x=s.x;
  const double center=std::atan2(x[2]+x[3]*dt-muzzle.y(),x[0]+x[1]*dt-muzzle.x());
  return wrap(predicted_plate(s,index,dt)[3]-center);
}
// Returns the plate to aim at; in_window=false means pre-aiming the next coming plate (do not fire).
int choose_plate(const SpatialState& s,const AimConfig& c,double dt,const Eigen::Vector3d& muzzle,int* lock,bool& in_window) {
  in_window=true;
  const bool outpost=s.model.kind==TargetKind::outpost;
  const double w=s.x[7];
  std::vector<double> delta(s.model.plates);
  std::vector<int> usable;
  for(int i=0;i<s.model.plates;++i) {
    delta[i]=facing(s,i,dt,muzzle);
    // An outpost plate height is only known after that plate was observed (RPS dz1/dz2).
    if(!outpost || s.plate_hits[i]>=c.outpost.min_plate_hits) usable.push_back(i);
  }
  if(std::abs(w)<=c.aimer.spin_threshold_radps && !outpost) {
    std::vector<int> ids;
    for(int i:usable) if(std::abs(delta[i])<=c.aimer.max_facing_angle_rad) ids.push_back(i);
    if(ids.empty()) {if(lock) *lock=-1; return -1;}
    if(ids.size()==1) {if(lock) *lock=-1; return ids[0];}
    // Lock prevents alternating between two plates both near 45 degrees.
    if(lock && std::find(ids.begin(),ids.end(),*lock)!=ids.end()) return *lock;
    const int id=*std::min_element(ids.begin(),ids.end(),[&](int a,int b){return std::abs(delta[a])<std::abs(delta[b]);});
    if(lock) *lock=id;
    return id;
  }
  if(lock) *lock=-1;
  const double coming=(outpost?c.outpost.coming_angle_deg:c.aimer.coming_angle_deg)*degree;
  const double leaving=(outpost?c.outpost.leaving_angle_deg:c.aimer.leaving_angle_deg)*degree;
  // A spinning target: plates entering view are more likely to be hit than plates leaving it.
  for(int i:usable) {
    if(std::abs(delta[i])>coming) continue;
    if(w>0 && delta[i]<leaving) return i;
    if(w<0 && delta[i]>-leaving) return i;
    if(w==0 && std::abs(delta[i])<=leaving) return i;
  }
  if(w==0) return -1;
  // Tongji returns no command in the gap between windows; pre-aim the plate that enters next instead,
  // so the gimbal is already waiting when it becomes fireable.
  in_window=false;
  int next=-1; double best=1e9;
  for(int i:usable) {
    const double gap=std::fmod(4*pi+(w>0?-coming-delta[i]:delta[i]-coming),2*pi);
    if(gap<best) {best=gap; next=i;}
  }
  return next;
}
}
AimSolution solve_aim(const SpatialState& state,const AimConfig& c,double age,const Eigen::Vector3d& muzzle,double speed,int* lock) {
  AimSolution result;
  if(!c.enabled) {result.status="disabled"; return result;}
  if(!state.valid) {result.status=state.status; return result;}
  if(c.timestamp_source=="host_receive") {result.status="exposure_time_unsynchronized"; return result;}
  if(!muzzle.allFinite() || !std::isfinite(age)||age<0||age>target_timeout_s(state,c)) {result.status="stale_or_invalid_time"; return result;}
  if(!(speed>0) || !std::isfinite(speed)) {result.status="no_muzzle_speed"; return result;}
  const bool fast=c.aimer.high_speed_threshold_radps>0 && std::abs(state.x[7])>c.aimer.high_speed_threshold_radps;
  const double delay=fast?c.aimer.high_speed_control_delay_s:c.aimer.control_delay_s;
  result.status="no_plate_in_window";
  double flight=0;
  for(int iteration=0;iteration<20;++iteration) {
    const double horizon=age+delay+flight;
    if(horizon>c.aimer.max_prediction_s) {result.status="prediction_too_long"; return result;}
    bool in_window=true;
    const int index=choose_plate(state,c,horizon,muzzle,lock,in_window);
    if(index<0) {result.status="no_plate_in_window"; return result;}
    const auto p=predicted_plate(state,index,horizon);
    const Eigen::Vector3d delta=p.head<3>()-muzzle;
    const double d=std::hypot(delta.x(),delta.y());
    const double v=speed;
    if(d<1e-6) {result.status="target_at_muzzle"; return result;}
    const double a=c.aimer.gravity_mps2*d*d/(2*v*v), z=a+delta.z();
    const double discriminant=d*d-4*a*z;
    if(discriminant<0 || !std::isfinite(discriminant)) {result.status="unreachable"; return result;}
    // Rationalized low-angle root avoids cancellation near a level trajectory.
    const double pitch=std::atan(2*z/(d+std::sqrt(discriminant)));
    const double next=d/(v*std::cos(pitch));
    if(!std::isfinite(next)) {result.status="unreachable"; return result;}
    if(std::abs(next-flight)<1e-4) {
      result.valid=true; result.plate=index; result.in_window=in_window;
      result.status=c.timestamp_source=="simulation"?"simulation_only":
                    c.timestamp_source=="host_compensated"?"solution_host_compensated":"solution";
      result.yaw_rad=wrap(std::atan2(delta.y(),delta.x())+c.aimer.yaw_offset_deg*degree);
      result.pitch_rad=pitch+c.aimer.pitch_offset_deg*degree;
      result.flight_time_s=next; result.point=p.head<3>(); result.distance_m=delta.norm();
      result.facing_rad=facing(state,index,horizon,muzzle);
      return result;
    }
    flight=next;
  }
  result.status="flight_time_not_converged";
  return result;
}
FireDecision Shooter::decide(const AimSolution& s,std::uint64_t track,const GimbalPose& g,bool enabled) {
  FireDecision d;
  const bool same=has_last_ && track==last_track_ && s.plate==last_plate_;
  const double jump=std::abs(wrap(s.yaw_rad-last_yaw_));
  has_last_=s.valid; last_track_=track; last_plate_=s.plate; last_yaw_=s.yaw_rad;
  if(!s.valid) {d.status=s.status; return d;}
  if(!s.in_window) {d.status="waiting_for_plate"; return d;}
  if(!config_.auto_fire) {d.status="auto_fire_off"; return d;}
  if(!enabled) {d.status="fire_not_enabled"; return d;}
  auto clamp=[&](double v){return std::clamp(v,config_.min_tolerance_rad,config_.max_tolerance_rad);};
  // Allowed pointing error: half the visible plate, as in RPS isGimbalAngleOk.
  const double yaw_tolerance=clamp(std::atan(config_.half_width_m*std::cos(s.facing_rad)/s.distance_m));
  const double pitch_tolerance=clamp(std::atan(config_.half_height_m/s.distance_m));
  if(!same || jump>2*yaw_tolerance) {d.status="command_changed"; return d;}
  if(std::abs(wrap(g.yaw_rad-s.yaw_rad))>yaw_tolerance || std::abs(g.pitch_rad-s.pitch_rad)>pitch_tolerance) {
    d.status="gimbal_not_on_target"; return d;
  }
  d.fire=true; d.status="fire";
  return d;
}
}
