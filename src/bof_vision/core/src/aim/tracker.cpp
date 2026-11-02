// Model geometry adapted from Tongji sp_vision_25 Target and RPS OutpostModel (MIT).
// See third_party/NOTICE.md and licenses/{TongjiSuperPower,RPS}-MIT.txt.
#include "bof_vision/aim.hpp"
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>

namespace bof {
namespace {
constexpr double pi=3.14159265358979323846;
double wrap(double x) { return std::remainder(x,2*pi); }
Eigen::MatrixXd jacobian(const SpatialState& s,int index) {
  const auto& x=s.x;
  const double a=x[6]+index*2*pi/s.model.plates;
  const bool odd=s.model.plates==4 && index%2;
  const double r=s.model.kind==TargetKind::outpost?s.model.radius_m:x[8]+(odd?x[9]:0);
  Eigen::MatrixXd H=Eigen::MatrixXd::Zero(4,x.size());
  H(0,0)=H(1,2)=H(2,4)=H(3,6)=1;
  H(0,6)=r*std::sin(a); H(1,6)=-r*std::cos(a);
  if(s.model.kind==TargetKind::outpost) { if(index>0) H(2,7+index)=1; }
  else {
    H(0,8)=-std::cos(a); H(1,8)=-std::sin(a);
    if(odd) {H(0,9)=-std::cos(a); H(1,9)=-std::sin(a); H(2,10)=1;}
  }
  return H;
}
}
const char* spin_name(Spin s) {
  switch(s) {
    case Spin::counter_clockwise: return "counter_clockwise";
    case Spin::clockwise: return "clockwise";
    case Spin::stopped: return "stopped";
    default: return "unknown";
  }
}
Spin vote_direction(const std::vector<double>& yaws,const std::vector<double>& times,int interval,double stable,double jump) {
  int anti=0,clockwise=0,still=0;
  for(std::size_t i=interval;i<yaws.size() && i<times.size();++i) {
    const double d=wrap(yaws[i]-yaws[i-interval]), dt=times[i]-times[i-interval];
    // Plate changes jump by 2pi/3 and are not rotation evidence.
    if(std::abs(d)>jump || !(dt>0)) continue;
    const double rate=d/dt;
    if(rate>stable) ++anti; else if(rate<-stable) ++clockwise; else ++still;
  }
  // RPS treated anti==clockwise before checking the stationary vote, so a still outpost was never
  // recognised; the still vote is checked first here.
  if(still>anti && still>clockwise) return Spin::stopped;
  if(anti>clockwise && anti>still) return Spin::counter_clockwise;
  if(clockwise>anti && clockwise>still) return Spin::clockwise;
  return Spin::unknown;
}
double target_timeout_s(const SpatialState& s,const AimConfig& c) {
  return s.model.kind==TargetKind::outpost?c.outpost.timeout_s:c.tracking.timeout_s;
}
Eigen::Vector4d predicted_plate(const SpatialState& s,int index,double dt) {
  if(s.x.size()<10 || index<0 || index>=s.model.plates || !std::isfinite(dt) || dt<0)
    throw std::invalid_argument("invalid plate prediction");
  const auto& x=s.x;
  const double a=wrap(x[6]+x[7]*dt+index*2*pi/s.model.plates);
  const bool odd=s.model.plates==4 && index%2;
  const double r=s.model.kind==TargetKind::outpost?s.model.radius_m:x[8]+(odd?x[9]:0);
  const double h=s.model.kind==TargetKind::outpost?(index?x[7+index]:0):(odd?x[10]:0);
  return {x[0]+x[1]*dt-r*std::cos(a),x[2]+x[3]*dt-r*std::sin(a),x[4]+x[5]*dt+h,a};
}
SpatialTracker::SpatialTracker(AimConfig config):config_(std::move(config)) {validate_aim_config(config_);}
void SpatialTracker::reset() {
  state_=SpatialState{}; covariance_.resize(0,0); last_seen_=0; hits_=0; better_frames_=0;
  confirmed_=false; vote_yaws_.clear(); vote_times_.clear(); pending_=Spin::unknown; disagreements_=0;
}
void SpatialTracker::set_policy(TargetPolicy policy) {policy_=std::move(policy);}
int SpatialTracker::rank(int label) const {
  if(label==policy_.priority_label) return -1;
  auto it=std::find(config_.selection.priority.begin(),config_.selection.priority.end(),label);
  return static_cast<int>(it-config_.selection.priority.begin());
}
void SpatialTracker::initialize(std::int64_t t,const SpatialObservation& o) {
  state_=SpatialState{};
  state_.model=config_.targets.at(o.label); state_.label=o.label; state_.id=++next_id_;
  const int n=state_.model.kind==TargetKind::outpost?10:11;
  state_.x=Eigen::VectorXd::Zero(n); auto& x=state_.x;
  x[0]=o.position.x()+state_.model.radius_m*std::cos(o.inward_yaw);
  x[2]=o.position.y()+state_.model.radius_m*std::sin(o.inward_yaw);
  x[4]=o.position.z(); x[6]=wrap(o.inward_yaw);
  if(n==11) x[8]=state_.model.radius_m;
  covariance_=Eigen::MatrixXd::Identity(n,n)*config_.tracking.initial_variance;
  state_.stamp_ns=t; last_seen_=t; hits_=1; better_frames_=0; confirmed_=false;
  vote_yaws_.clear(); vote_times_.clear(); pending_=Spin::unknown; disagreements_=0;
  state_.matched_plate=0; state_.plate_hits[0]=1; state_.status="detecting";
  pin_spin();
}
void SpatialTracker::vote(double yaw) {
  vote_yaws_.push_back(yaw); vote_times_.push_back(state_.stamp_ns*1e-9);
  if(static_cast<int>(vote_yaws_.size())<config_.outpost.vote_samples) return;
  const Spin s=vote_direction(vote_yaws_,vote_times_,config_.outpost.vote_interval,config_.outpost.vote_stable_radps,config_.outpost.vote_jump_rad);
  vote_yaws_.clear(); vote_times_.clear();
  if(s==Spin::unknown) return;
  if(state_.spin==Spin::unknown) {state_.spin=s; return;}
  // The direction is fixed per match but the outpost stops later; require two agreeing windows to change.
  if(s==state_.spin) {disagreements_=0; return;}
  if(s!=pending_) {pending_=s; disagreements_=0;}
  if(++disagreements_>=2) {state_.spin=s; disagreements_=0;}
}
void SpatialTracker::pin_spin() {
  if(state_.model.kind!=TargetKind::outpost) return;
  if(policy_.outpost_stopped || config_.outpost.spin=="stopped") state_.spin=Spin::stopped;
  if(state_.spin==Spin::unknown) return;
  double w=0;
  if(state_.spin!=Spin::stopped) {
    const double sign=state_.spin==Spin::counter_clockwise?1:-1;
    // Without a rule speed only the sign is enforced and the filter keeps estimating the magnitude.
    if(!(config_.outpost.speed_radps>0)) {state_.x[7]=sign*std::abs(state_.x[7]); return;}
    w=sign*config_.outpost.speed_radps;
  }
  state_.x[7]=w;
  covariance_.row(7).setZero(); covariance_.col(7).setZero(); covariance_(7,7)=1e-6;
}
SpatialState SpatialTracker::update(std::int64_t t,const std::vector<SpatialObservation>& input) {
  if(t<=0 || (state_.stamp_ns && t<=state_.stamp_ns)) {reset(); state_.status="nonmonotonic_time"; return state_;}
  std::vector<SpatialObservation> obs;
  for(const auto& o:input)
    if(config_.targets.count(o.label)&&o.position.allFinite()&&std::isfinite(o.inward_yaw)&&!policy_.excluded.count(o.label)&&
       (policy_.lock_label<0||o.label==policy_.lock_label)) obs.push_back(o);
  if(state_.x.size() && (t-last_seen_)*1e-9>target_timeout_s(state_,config_)) reset();
  if(state_.x.size() && (policy_.excluded.count(state_.label) || (policy_.lock_label>=0 && state_.label!=policy_.lock_label))) {
    reset(); state_.status="policy_released";
  }
  auto best=std::min_element(obs.begin(),obs.end(),[&](const auto& a,const auto& b){
    const int ra=rank(a.label),rb=rank(b.label);
    return ra!=rb?ra<rb:a.position.squaredNorm()<b.position.squaredNorm();});
  if(!state_.x.size()) {
    if(obs.empty()) {if(state_.status!="policy_released") state_.status="no_supported_observation"; return state_;}
    initialize(t,*best); return state_;
  }
  // Tongji Tracker switches when a higher-priority target is visible; persistence avoids flicker.
  if(best!=obs.end() && rank(best->label)<rank(state_.label)) {
    if(++better_frames_>=config_.selection.switch_frames) {initialize(t,*best); return state_;}
  } else better_frames_=0;
  const double dt=(t-state_.stamp_ns)*1e-9;
  const int n=state_.x.size();
  Eigen::MatrixXd F=Eigen::MatrixXd::Identity(n,n), Q=Eigen::MatrixXd::Zero(n,n);
  for(int i:{0,2,4,6}) {
    F(i,i+1)=dt;
    const double variance=i==6?config_.tracking.angular_acceleration_variance:config_.tracking.acceleration_variance;
    Q(i,i)=std::pow(dt,4)/4*variance; Q(i,i+1)=Q(i+1,i)=std::pow(dt,3)/2*variance;
    Q(i+1,i+1)=dt*dt*variance;
  }
  for(int i=8;i<n;++i) Q(i,i)=config_.tracking.shape_variance*dt;
  state_.x=F*state_.x; state_.x[6]=wrap(state_.x[6]);
  covariance_=F*covariance_*F.transpose()+Q;
  pin_spin();
  state_.stamp_ns=t; state_.matched_plate=-1;
  double best_score=std::numeric_limits<double>::infinity(),second=best_score;
  int plate=-1; const SpatialObservation* chosen=nullptr;
  struct Match {double score; int plate; const SpatialObservation* observation;};
  std::vector<Match> matches;
  for(const auto& o:obs) {
    if(o.label!=state_.label) continue;
    for(int i=0;i<state_.model.plates;++i) {
      auto prediction=predicted_plate(state_,i);
      double distance=(prediction.head<3>()-o.position).norm(),angle=std::abs(wrap(prediction[3]-o.inward_yaw));
      if(distance>config_.tracking.position_gate_m || angle>config_.tracking.yaw_gate_rad) continue;
      double score=std::hypot(distance/config_.tracking.position_gate_m,angle/config_.tracking.yaw_gate_rad);
      matches.push_back({score,i,&o});
      if(score<best_score) {best_score=score; plate=i; chosen=&o;}
    }
  }
  // Different observations matching different plates are compatible evidence.
  // Ambiguity only concerns a competing identity for this observation or plate.
  for(const auto& m:matches)
    if((m.observation==chosen && m.plate!=plate) || (m.observation!=chosen && m.plate==plate)) second=std::min(second,m.score);
  if(!chosen || second-best_score<config_.tracking.ambiguity_margin) {
    // Tongji temp_lost: a confirmed target keeps predicting until the timeout.
    state_.valid=confirmed_; if(!confirmed_) hits_=0;
    state_.status=chosen?"association_ambiguous":"temporarily_lost"; return state_;
  }
  const auto H=jacobian(state_,plate);
  Eigen::Vector4d residual;
  residual.head<3>()=chosen->position; residual[3]=chosen->inward_yaw;
  residual-=predicted_plate(state_,plate); residual[3]=wrap(residual[3]);
  Eigen::Matrix4d R=Eigen::Matrix4d::Identity()*config_.tracking.position_variance; R(3,3)=config_.tracking.yaw_variance;
  Eigen::Matrix4d S=H*covariance_*H.transpose()+R;
  Eigen::LDLT<Eigen::Matrix4d> decomposition(S);
  if(decomposition.info()!=Eigen::Success || !decomposition.isPositive()) {reset(); state_.status="singular_filter"; return state_;}
  Eigen::MatrixXd K=decomposition.solve(H*covariance_).transpose();
  state_.x+=K*residual; state_.x[6]=wrap(state_.x[6]);
  Eigen::MatrixXd I=Eigen::MatrixXd::Identity(n,n)-K*H;
  covariance_=I*covariance_*I.transpose()+K*R*K.transpose();
  covariance_=(0.5*(covariance_+covariance_.transpose())).eval();
  if(state_.model.kind==TargetKind::outpost) {vote(chosen->inward_yaw); pin_spin();}
  bool shape_ok=state_.x.allFinite()&&covariance_.allFinite();
  if(n==11) {
    const double r1=state_.x[8],r2=r1+state_.x[9];
    shape_ok=shape_ok&&r1>=config_.tracking.min_radius_m&&r1<=config_.tracking.max_radius_m&&
      r2>=config_.tracking.min_radius_m&&r2<=config_.tracking.max_radius_m&&std::abs(state_.x[10])<=config_.tracking.max_height_difference_m;
  } else shape_ok=shape_ok&&std::abs(state_.x[8])<=config_.tracking.max_height_difference_m&&std::abs(state_.x[9])<=config_.tracking.max_height_difference_m;
  if(!shape_ok) {reset(); state_.status="invalid_target_geometry"; return state_;}
  last_seen_=t; state_.matched_plate=plate; ++state_.plate_hits[plate];
  confirmed_=confirmed_||++hits_>=config_.tracking.confirm_frames;
  state_.valid=confirmed_; state_.status=state_.valid?"tracking":"detecting";
  return state_;
}
}
