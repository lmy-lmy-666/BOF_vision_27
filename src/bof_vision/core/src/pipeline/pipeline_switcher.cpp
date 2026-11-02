#include "bof_vision/pipeline_switcher.hpp"
#include <chrono>
#include <stdexcept>

namespace bof {
std::string PipelineSwitcher::key(const DetectorConfig& d) {
  return d.backend+'\n'+d.model_path+'\n'+d.device;
}
std::unique_ptr<PerceptionPipeline> PipelineSwitcher::build(PerceptionConfig config) {
  validate_config(config);
  auto pipeline=std::make_unique<PerceptionPipeline>(config.detector,config.track_timeout_s,
    config.association_gate_px,config.pose,config.association);
  pipeline->process({cv::Mat::zeros(480,640,CV_8UC3),0,0});
  pipeline->reset();
  return pipeline;
}
PipelineSwitcher::PipelineSwitcher(PerceptionConfig config) : config_(std::move(config)) {
  validate_config(config_); pipeline_=build(config_);
}
bool PipelineSwitcher::request(const std::string& name) {
  if(pending_) return false;
  error_.clear(); target_=config_;
  try {
    select_yolo(target_,name); validate_config(target_);
    const auto wanted=key(target_.detector);
    reuse_current_=wanted==key(config_.detector); cached_=reuse_current_;
    if(!reuse_current_) {
      for(auto it=cache_.begin();it!=cache_.end();++it) if(it->first==wanted) {
        ready_=std::move(it->second); cache_.erase(it); cached_=true; break;
      }
      if(!ready_) loading_=std::async(std::launch::async,[config=target_]{return build(config);});
    }
    pending_=true; return true;
  } catch(const std::exception& error) { error_=error.what(); return false; }
}
SwitchResult PipelineSwitcher::poll() {
  if(!pending_) return SwitchResult::none;
  if(!reuse_current_ && !ready_) {
    if(loading_.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready) return SwitchResult::none;
    try { ready_=loading_.get(); }
    catch(const std::exception& error) { error_=error.what(); pending_=false; return SwitchResult::failed; }
  }
  auto& next=reuse_current_ ? pipeline_ : ready_;
  try {
    const auto& d=target_.detector;
    next->set_detection_parameters(d.enemy_color,d.color_threshold,d.confidence_threshold,d.nms_threshold);
  } catch(const std::exception& error) { error_=error.what(); ready_.reset(); pending_=false; return SwitchResult::failed; }
  if(!reuse_current_) {
    if(config_.live.model_cache_size>0) cache_.emplace_front(key(config_.detector),std::move(pipeline_));
    pipeline_=std::move(ready_);
    while(cache_.size()>static_cast<std::size_t>(config_.live.model_cache_size)) cache_.pop_back();
  }
  config_=target_; ++revision_; pending_=false;
  return SwitchResult::applied;
}
void PipelineSwitcher::set_detection_parameters(const DetectorConfig& d) {
  if(pending_) throw std::logic_error("wait for pending model switch before tuning detection");
  if(key(d)!=key(config_.detector)) throw std::invalid_argument("use a named profile to change model/backend/device");
  pipeline_->set_detection_parameters(d.enemy_color,d.color_threshold,d.confidence_threshold,d.nms_threshold);
  config_.detector=d;
}
}  // namespace bof
