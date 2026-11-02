// Release/reopen recovery adapted from Tongji io/hikrobot/hikrobot.cpp (MIT).
// Copyright (c) 2025 TongjiSuperPower; see third_party/licenses/TongjiSuperPower-MIT.txt.
#include "bof_vision/camera.hpp"
#include "bof_vision/config.hpp"
#include "diagnostics.hpp"
#include <atomic>
#include <cmath>
#include <memory>
#include <thread>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int64.hpp>

class HikCamera : public rclcpp::Node {
public:
  HikCamera() : Node("hik_camera") {
    rcl_interfaces::msg::ParameterDescriptor fixed; fixed.read_only=true;
    const auto config=bof::load_config(declare_parameter<std::string>("config_file","",fixed));
    frame_id_=declare_parameter<std::string>("frame_id","camera_optical_frame",fixed);
    serial_=declare_parameter<std::string>("serial_number",config.live.camera_serial,fixed);
    if(serial_=="auto") serial_.clear();
    exposure_=declare_parameter<double>("exposure_us",config.live.exposure_us,fixed);
    gain_=declare_parameter<double>("gain",config.live.gain,fixed);
    reconnect_=declare_parameter<bool>("reconnect",config.live.reconnect,fixed);
    deadline_=declare_parameter<double>("reconnect_timeout_s",config.live.reconnect_timeout_s,fixed);
    delay_=declare_parameter<int>("retry_delay_ms",config.live.retry_delay_ms,fixed);
    if(!std::isfinite(exposure_) || exposure_<=0 || !std::isfinite(gain_) || gain_<0 ||
       !std::isfinite(deadline_) || deadline_<0 || delay_<50) throw std::invalid_argument("invalid camera recovery parameters");
    pub_=create_publisher<sensor_msgs::msg::Image>("camera/image_raw",rclcpp::SensorDataQoS().keep_last(2));
    generation_pub_=create_publisher<std_msgs::msg::UInt64>("camera/generation",rclcpp::QoS(1).transient_local());
    diagnostics_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics",10);
    worker_=std::thread([this]{
      try { capture(); }
      catch(const std::exception& error) {
        // Publishing can fail when the ROS context shuts down during recovery.
        if(rclcpp::ok()) {
          failed_=true; RCLCPP_ERROR(get_logger(),"camera worker stopped: %s",error.what());
          rclcpp::shutdown();
        }
      }
    });
  }
  ~HikCamera() override { stop_=true; if(worker_.joinable()) worker_.join(); }
private:
  using Clock=std::chrono::steady_clock;
  void report(int level,const std::string& state,const std::string& error="") {
    publish_diagnostic(diagnostics_,now(),"bof/camera",level,state,{
      {"serial",serial_},{"generation",std::to_string(generation_)},{"frames",std::to_string(frames_)},
      {"device_frame_gaps",std::to_string(gaps_)},{"failures",std::to_string(failures_)},
      {"timestamp_source","host_receive (not exposure synchronized)"},{"last_error",error}});
  }
  void capture() {
    std::unique_ptr<bof::CameraSource> camera;
    std::optional<Clock::time_point> outage;
    std::optional<std::uint64_t> last_sequence;
    auto last_report=Clock::now(); unsigned timeouts=0; bool first=true;
    while(!stop_ && rclcpp::ok()) {
      try {
        if(!camera) {
          report(1,"connecting");
          camera=bof::make_hik_camera(serial_,exposure_,gain_);
          if(serial_.empty()) serial_=camera->serial_number();
          ++generation_; first=true; timeouts=0; last_sequence.reset();
        }
        auto frame=camera->read(200);
        if(!frame) { if(++timeouts>=10) throw std::runtime_error("10 consecutive frame timeouts"); continue; }
        timeouts=0; outage.reset(); ++frames_;
        if(last_sequence && frame->sequence>*last_sequence+1) gaps_+=frame->sequence-*last_sequence-1;
        last_sequence=frame->sequence;
        if(first) {
          std_msgs::msg::UInt64 generation; generation.data=generation_; generation_pub_->publish(generation);
          first=false; report(0,"streaming");
        }
        std_msgs::msg::Header header; header.frame_id=frame_id_;
        const auto age=std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now()-frame->received).count();
        header.stamp=now()-rclcpp::Duration::from_nanoseconds(age);
        pub_->publish(*cv_bridge::CvImage(header,"bgr8",frame->bgr).toImageMsg());
        if(Clock::now()-last_report>=std::chrono::seconds(1)) { report(0,"streaming"); last_report=Clock::now(); }
      } catch(const std::exception& error) {
        if(stop_ || !rclcpp::ok()) break;
        camera.reset(); ++failures_;
        if(!outage) outage=Clock::now();
        const bool expired=deadline_>0 && Clock::now()-*outage>=std::chrono::duration<double>(deadline_);
        report(2,(!reconnect_ || expired) ? "stopped" : "reconnecting",error.what());
        RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"camera unavailable: %s",error.what());
        if(!reconnect_ || expired) { failed_=true; rclcpp::shutdown(); break; }
        const auto retry=Clock::now()+std::chrono::milliseconds(delay_);
        while(!stop_ && rclcpp::ok() && Clock::now()<retry) std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }
public:
  bool failed() const { return failed_; }
private:
  std::string frame_id_,serial_;
  double exposure_=0,gain_=0,deadline_=0; int delay_=1000; bool reconnect_=true;
  std::uint64_t generation_=0,frames_=0,gaps_=0,failures_=0;
  std::atomic<bool> stop_{false},failed_{false}; std::thread worker_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr generation_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv); int result=0;
  try { auto node=std::make_shared<HikCamera>(); rclcpp::spin(node); result=node->failed() ? 1 : 0; }
  catch(const std::exception& error) { RCLCPP_ERROR(rclcpp::get_logger("hik_camera"),"%s",error.what()); result=1; }
  if(rclcpp::ok()) rclcpp::shutdown();
  return result;
}
