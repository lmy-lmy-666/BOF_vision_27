#pragma once
// Adapted from SZURPVision LatestBuffer (MIT); removes Context dependency and adds close().
// Copyright (c) 2026 SZURPVision. See third_party/licenses/SZURPVision-MIT.txt.
#include <condition_variable>
#include <mutex>
#include <optional>
#include <utility>
#include <cstdint>
#include <chrono>

namespace bof {
template<class T> class LatestBuffer {
public:
  bool push(T value) {
    { std::lock_guard<std::mutex> lock(mutex_);
      if (closed_) return false;
      if (value_) ++overwritten_;
      value_=std::move(value); }
    ready_.notify_one(); return true;
  }
  std::optional<T> wait_pop() {
    std::unique_lock<std::mutex> lock(mutex_);
    ready_.wait(lock,[this]{return closed_ || value_.has_value();});
    auto result=std::move(value_); value_.reset(); return result;
  }
  template<class Rep,class Period>
  std::optional<T> wait_pop_for(std::chrono::duration<Rep,Period> timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(!ready_.wait_for(lock,timeout,[this]{return closed_ || value_.has_value();})) return std::nullopt;
    auto result=std::move(value_); value_.reset(); return result;
  }
  void discard() { std::lock_guard<std::mutex> lock(mutex_); value_.reset(); }
  bool closed() const { std::lock_guard<std::mutex> lock(mutex_); return closed_; }
  void close() {
    { std::lock_guard<std::mutex> lock(mutex_); closed_=true; }
    ready_.notify_all();
  }
  std::uint64_t overwritten() const {
    std::lock_guard<std::mutex> lock(mutex_); return overwritten_;
  }
private:
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::optional<T> value_;
  bool closed_=false;
  std::uint64_t overwritten_=0;
};
}  // namespace bof
