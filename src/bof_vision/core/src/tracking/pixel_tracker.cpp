// Detecting/tracking/temporary-loss state flow adapted from Tongji
// tasks/auto_aim/tracker.cpp (MIT), Copyright (c) 2025 TongjiSuperPower.
// See third_party/licenses/TongjiSuperPower-MIT.txt.
// Image-space association uses OpenCV KalmanFilter, not the upstream 3D target EKF.
#include "bof_vision/core.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace bof {
PixelTracker::PixelTracker(double timeout_s, double gate_px, const AssociationConfig& config)
  : timeout_(timeout_s), gate_(gate_px), config_(config) {
  if(!std::isfinite(timeout_) || !std::isfinite(gate_) || timeout_<=0 || gate_<=0)
    throw std::invalid_argument("tracker timeout and gate must be finite and positive");
  if(config_.confirm_frames<1 || config_.confirm_frames>1000 ||
     config_.label_confirm_frames<1 || config_.label_confirm_frames>1000 ||
     !std::isfinite(config_.ambiguity_px) || config_.ambiguity_px<0 || config_.ambiguity_px>=gate_)
    throw std::invalid_argument("tracker confirmation counts must be 1..1000 and ambiguity margin in [0,gate)");
}
void PixelTracker::reset() {
  active_=false; confirmed_=false; hits_=0; pending_hits_=0;
  last_stamp_.reset(); last_seen_.reset(); label_=pending_label_=ArmorLabel::unknown;
}
void PixelTracker::start(const Candidate& candidate, double stamp, int hits) {
  filter_.init(4,2,0,CV_64F);  // x, y, vx, vy; measurements x, y
  cv::setIdentity(filter_.transitionMatrix);
  filter_.measurementMatrix.at<double>(0,0)=1;
  filter_.measurementMatrix.at<double>(1,1)=1;
  cv::setIdentity(filter_.measurementNoiseCov,cv::Scalar(16));
  cv::setIdentity(filter_.errorCovPost,cv::Scalar(16));
  filter_.errorCovPost.at<double>(2,2)=filter_.errorCovPost.at<double>(3,3)=1e6;
  const auto center=candidate.center();
  filter_.statePost.at<double>(0)=center.x; filter_.statePost.at<double>(1)=center.y;
  active_=true; hits_=hits; confirmed_=hits_>=config_.confirm_frames;
  pending_hits_=0; pending_label_=ArmorLabel::unknown;
  last_seen_=stamp; label_=candidate.label; ++id_;
}
PixelTrack PixelTracker::update(const std::vector<Candidate>& detections, double stamp) {
  if(!std::isfinite(stamp)) { reset(); PixelTrack out; out.status="invalid_timestamp"; return out; }
  if((last_stamp_ && stamp<=*last_stamp_) || (last_seen_ && stamp-*last_seen_>timeout_)) reset();
  const double dt=last_stamp_ ? stamp-*last_stamp_ : 0;
  last_stamp_=stamp;
  cv::Point2d expected{};
  if(active_) {
    filter_.transitionMatrix.at<double>(0,2)=filter_.transitionMatrix.at<double>(1,3)=dt;
    // Continuous white acceleration model. Pixel-domain tuning, not physical dynamics.
    const double q=250000, p=q*dt*dt*dt/3, c=q*dt*dt/2, v=q*dt;
    filter_.processNoiseCov=(cv::Mat_<double>(4,4)<<p,0,c,0, 0,p,0,c, c,0,v,0, 0,c,0,v);
    const auto prediction=filter_.predict();
    if(!cv::checkRange(prediction)) { reset(); PixelTrack out; out.status="invalid_filter"; return out; }
    expected={prediction.at<double>(0),prediction.at<double>(1)};
  }
  auto missing=[&](const char* reason) {
    pending_hits_=0; pending_label_=ArmorLabel::unknown;
    if(!confirmed_) {
      reset();
      if(std::string(reason)=="temp_lost") reason="lost";
    }
    PixelTrack out; out.status=reason;
    if(active_) { out.id=id_; out.label=label_; }
    return out;
  };
  int best=-1;
  double error=gate_, second=std::numeric_limits<double>::infinity();
  float score=-std::numeric_limits<float>::infinity();
  for(std::size_t i=0;i<detections.size();++i) {
    const auto center=detections[i].center();
    if(!std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(detections[i].score)) continue;
    if(!active_) {
      if(detections[i].score>score) { score=detections[i].score; best=static_cast<int>(i); }
    } else {
      const double distance=cv::norm(cv::Point2d(center)-expected);
      if(distance>=gate_) continue;
      if(distance<error) { second=error; error=distance; best=static_cast<int>(i); }
      else if(distance<second) second=distance;
    }
  }
  if(best<0) return missing(detections.empty() ? (active_ ? "temp_lost" : "lost") : "outside_gate");
  // Only actual candidates count as alternatives; gate_ is not a second candidate.
  if(active_ && second<gate_ && second-error<=config_.ambiguity_px) return missing("ambiguous");
  const auto& candidate=detections[static_cast<std::size_t>(best)];
  const bool was_active=active_;
  if(!active_ || (!confirmed_ && candidate.label!=label_)) start(candidate,stamp);
  else if(candidate.label!=label_) {
    if(candidate.label==ArmorLabel::unknown) { pending_hits_=0; pending_label_=ArmorLabel::unknown; }
    else if(candidate.label==pending_label_) ++pending_hits_;
    else { pending_label_=candidate.label; pending_hits_=1; }
    if(pending_hits_>=config_.label_confirm_frames) {
      // A confirmed semantic change starts a new identity and motion model.
      start(candidate,stamp,pending_hits_);
    } else {
      PixelTrack out; out.id=id_; out.label=label_; out.observed=true;
      out.candidate_index=best; out.observed_label=candidate.label;
      out.association_error_px=error; out.status="label_pending";
      return out;  // Do not update motion or extend lifetime on contradictory labels.
    }
  } else {
    const auto center=candidate.center();
    filter_.correct((cv::Mat_<double>(2,1)<<center.x,center.y));
    last_seen_=stamp; pending_hits_=0; pending_label_=ArmorLabel::unknown;
    if(hits_<config_.confirm_frames) ++hits_;
    confirmed_=hits_>=config_.confirm_frames;
  }
  PixelTrack out; out.valid=confirmed_; out.id=id_; out.label=label_;
  out.observed=true; out.candidate_index=best; out.observed_label=candidate.label;
  out.status=confirmed_ ? "tracking" : "detecting";
  out.association_error_px=was_active ? error : -1;
  if(out.valid) out.center=candidate.center();  // Always the current measurement, never prediction.
  return out;
}
}  // namespace bof
