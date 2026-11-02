#pragma once
// Adapted from TongjiSuperPower/sp_vision_25 tools/recorder (MIT).
// Copyright (c) 2025 TongjiSuperPower. See third_party/licenses/TongjiSuperPower-MIT.txt.
#include "bof_vision/latest_buffer.hpp"
#include <atomic>
#include <chrono>
#include <exception>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <opencv2/videoio.hpp>

namespace bof {
// Async MJPG recorder, sampled at at most fps; never blocks inference on disk I/O.
// frames.csv maps the actual saved AVI frame index back to source timestamps.
class Recorder {
public:
  Recorder(const std::string& directory,double fps);
  ~Recorder();
  void record(const cv::Mat& bgr,std::uint64_t sequence,std::int64_t stamp_ns,
              std::chrono::steady_clock::time_point received);
  void close();
  void check_error() const;
  std::uint64_t saved() const { return saved_.load(); }
  std::uint64_t overwritten() const { return queue_.overwritten(); }
private:
  struct FrameData {
    cv::Mat image;
    std::uint64_t sequence;
    std::int64_t stamp_ns;
    std::chrono::steady_clock::time_point received;
  };
  void save_to_file();
  std::string directory_;
  double fps_;
  std::optional<std::chrono::steady_clock::time_point> last_time_;
  LatestBuffer<FrameData> queue_;
  std::atomic<std::uint64_t> saved_{0};
  std::thread worker_;
  mutable std::mutex error_mutex_;
  std::exception_ptr error_;
};
}  // namespace bof
