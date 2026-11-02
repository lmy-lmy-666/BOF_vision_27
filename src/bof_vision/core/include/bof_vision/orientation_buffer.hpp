#pragma once
// Interpolation design adapted from Alliance ImuSnapshotBuffer (MIT).
// Copyright (c) 2025 Alliance Algorithm Team. See third_party/licenses/Alliance-MIT.txt.
// Standalone version: explicit clock domain, bounded capacity and interpolation gap.
#include "bof_vision/pipeline.hpp"
#include <algorithm>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <Eigen/Geometry>

namespace bof {
struct OrientationSample {
  std::int64_t stamp_ns;
  Eigen::Quaterniond orientation;
};
class OrientationBuffer {
public:
  OrientationBuffer(ClockDomain clock, std::int64_t max_gap_ns, std::size_t capacity=512)
  : clock_(clock), gap_(max_gap_ns), capacity_(capacity) {
    if (gap_<=0 || capacity_<2) throw std::invalid_argument("invalid orientation buffer bounds");
  }
  bool push(OrientationSample sample) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!sample.orientation.coeffs().allFinite() || sample.orientation.norm()<1e-9 ||
        (!samples_.empty() && sample.stamp_ns<=samples_.back().stamp_ns)) return false;
    sample.orientation.normalize(); samples_.push_back(sample);
    if(samples_.size()>capacity_) samples_.pop_front();
    return true;
  }
  std::optional<Eigen::Quaterniond> at(std::int64_t stamp, ClockDomain clock) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if(clock!=clock_ || samples_.empty()) return std::nullopt;
    auto right=std::lower_bound(samples_.begin(),samples_.end(),stamp,
      [](const auto& item,std::int64_t t){return item.stamp_ns<t;});
    if(right==samples_.end()) return std::nullopt;
    if(right->stamp_ns==stamp) return right->orientation;
    if(right==samples_.begin()) return std::nullopt;
    auto left=std::prev(right);
    const long double span=static_cast<long double>(right->stamp_ns)-left->stamp_ns;
    if(span>gap_) return std::nullopt;
    const double ratio=static_cast<double>((static_cast<long double>(stamp)-left->stamp_ns)/span);
    return left->orientation.slerp(ratio,right->orientation).normalized();
  }
  void reset() { std::lock_guard<std::mutex> lock(mutex_); samples_.clear(); }
private:
  ClockDomain clock_;
  std::int64_t gap_;
  std::size_t capacity_;
  mutable std::mutex mutex_;
  std::deque<OrientationSample> samples_;
};
}  // namespace bof
