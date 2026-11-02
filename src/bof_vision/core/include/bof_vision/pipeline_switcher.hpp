#pragma once
#include "bof_vision/config.hpp"
#include "bof_vision/pipeline.hpp"
#include <future>
#include <list>

namespace bof {
enum class SwitchResult { none, applied, failed };
// Single processing-thread API. Cold models are built by one background worker;
// each pipeline owns its own inference request and is never processed concurrently.
class PipelineSwitcher {
public:
  explicit PipelineSwitcher(PerceptionConfig config);
  bool request(const std::string& name);
  SwitchResult poll();  // Call between frames; failures retain the current pipeline.
  PerceptionResult process(const Frame& frame) { return pipeline_->process(frame); }
  void reset() { pipeline_->reset(); }
  void set_detection_parameters(const DetectorConfig& detector);
  void set_live_parameters(const LiveConfig& live) { config_.live=live; if(pending_) target_.live=live; }
  const PerceptionConfig& config() const { return config_; }
  bool busy() const { return pending_; }
  const std::string& error() const { return error_; }
  std::string pending_name() const { return pending_ ? target_.yolo_name : ""; }
  bool cache_hit() const { return cached_; }
  std::uint64_t revision() const { return revision_; }
private:
  static std::unique_ptr<PerceptionPipeline> build(PerceptionConfig config);
  static std::string key(const DetectorConfig& config);
  PerceptionConfig config_,target_;
  std::unique_ptr<PerceptionPipeline> pipeline_,ready_;
  std::list<std::pair<std::string,std::unique_ptr<PerceptionPipeline>>> cache_;
  std::future<std::unique_ptr<PerceptionPipeline>> loading_;
  bool pending_=false,cached_=false,reuse_current_=false;
  std::string error_;
  std::uint64_t revision_=0;
};
}  // namespace bof
