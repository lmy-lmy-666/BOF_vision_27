// Adapted from TongjiSuperPower/sp_vision_25 tools/recorder.cpp (MIT).
// Copyright (c) 2025 TongjiSuperPower. See third_party/licenses/TongjiSuperPower-MIT.txt.
// Keeps async MJPG + metadata and rate-limiting; replaces upstream queue with our
// SZURPVision-derived closeable latest buffer. No fabricated IMU quaternion.
#include "bof_vision/recorder.hpp"
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace bof {
Recorder::Recorder(const std::string& directory,double fps) : directory_(directory),fps_(fps) {
  if(!std::isfinite(fps) || fps<=0 || fps>1000) throw std::invalid_argument("record FPS must be in (0,1000]");
  if(!std::filesystem::create_directory(directory_)) throw std::runtime_error("recording directory already exists: "+directory_);
  worker_=std::thread(&Recorder::save_to_file,this);
}
Recorder::~Recorder() {
  queue_.close();
  if(worker_.joinable()) worker_.join();
}
void Recorder::check_error() const {
  std::lock_guard<std::mutex> lock(error_mutex_);
  if(error_) std::rethrow_exception(error_);
}
void Recorder::close() {
  queue_.close();
  if(worker_.joinable()) worker_.join();
  check_error();
}
void Recorder::record(const cv::Mat& image,std::uint64_t sequence,std::int64_t stamp_ns,
                      std::chrono::steady_clock::time_point received) {
  check_error();
  if(image.empty()) return;
  if(last_time_ && std::chrono::duration<double>(received-*last_time_).count()<1.0/fps_) return;
  last_time_=received;
  queue_.push({image.clone(),sequence,stamp_ns,received});
}
void Recorder::save_to_file() {
  try {
    cv::VideoWriter writer;
    cv::Size size;
    std::ofstream metadata(directory_+"/frames.csv");
    if(!metadata) throw std::runtime_error("cannot open recording metadata");
    metadata << "video_index,source_sequence,source_stamp_ns,host_received_steady_ns\n";
    auto last_space_check=std::chrono::steady_clock::time_point::min();
    while(auto frame=queue_.wait_pop()) {
      const auto now=std::chrono::steady_clock::now();
      if(last_space_check==std::chrono::steady_clock::time_point::min() ||
         now-last_space_check>std::chrono::seconds(1)) {
        if(std::filesystem::space(directory_).available<512ULL*1024*1024)
          throw std::runtime_error("recording stopped: less than 512 MiB free disk space");
        last_space_check=now;
      }
      if(!writer.isOpened()) {
        size=frame->image.size();
        if(!writer.open(directory_+"/raw.avi",cv::VideoWriter::fourcc('M','J','P','G'),fps_,size))
          throw std::runtime_error("cannot open MJPG video writer");
      }
      if(frame->image.size()!=size) throw std::runtime_error("recording resolution changed; start a new recording");
      writer.write(frame->image);
      metadata << saved_.load() << ',' << frame->sequence << ',' << frame->stamp_ns << ','
        << std::chrono::duration_cast<std::chrono::nanoseconds>(frame->received.time_since_epoch()).count() << '\n';
      metadata.flush();
      if(!metadata) throw std::runtime_error("recording metadata write failed");
      ++saved_;
    }
    writer.release();
  } catch(...) {
    { std::lock_guard<std::mutex> lock(error_mutex_); error_=std::current_exception(); }
    queue_.close();
  }
}
}  // namespace bof
