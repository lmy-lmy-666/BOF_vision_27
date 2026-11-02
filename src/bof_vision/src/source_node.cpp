#include "bof_vision/core.hpp"
#include "bof_vision/config.hpp"
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class Source : public rclcpp::Node {
public:
  Source() : Node("source") {
    auto config = bof::load_config(declare_parameter<std::string>("config_file", ""));
    mode_ = declare_parameter<std::string>("mode", "synthetic");
    auto path = declare_parameter<std::string>("video_path", config.live.video_path);
    frame_ = declare_parameter<std::string>("frame_id", "camera_optical_frame");
    auto color = declare_parameter<std::string>("synthetic_color", bof::color_name(config.detector.enemy_color));
    if (color != "red" && color != "blue") throw std::invalid_argument("synthetic_color: red or blue");
    color_ = color == "red" ? bof::Color::red : bof::Color::blue;
    double fps = declare_parameter<double>("fps", config.live.fps);
    if (mode_ == "video") {
      if (path.empty() || !video_.open(path)) throw std::runtime_error("cannot open video: " + path);
      if (fps == 0.0) fps = video_.get(cv::CAP_PROP_FPS);
    } else if (mode_ == "synthetic") {
      if (fps == 0.0) fps = 30.0;
    } else throw std::invalid_argument("source mode must be synthetic or video");
    if (!std::isfinite(fps) || fps < 1.0 || fps > 1000.0) throw std::invalid_argument("invalid fps; set fps explicitly");
    period_ = rclcpp::Duration::from_seconds(1.0/fps);
    epoch_ = now();
    pub_ = create_publisher<sensor_msgs::msg::Image>("camera/image_raw", rclcpp::SensorDataQoS().keep_last(2));
    timer_ = create_wall_timer(std::chrono::nanoseconds(period_.nanoseconds()), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "source=%s fps=%.2f; replay stamps use nominal frame time", mode_.c_str(), fps);
  }
private:
  void tick() {
    cv::Mat image;
    if (mode_ == "video") {
      if (!video_.read(image) || image.empty()) {
        RCLCPP_INFO(get_logger(), "end of video after %ld frames", static_cast<long>(index_));
        timer_->cancel(); return;
      }
    } else image = bof::synthetic_frame(static_cast<int>(index_), color_);
    std_msgs::msg::Header header;
    header.frame_id = frame_;
    // A normal video has no exposure timestamps. Use a repeatable nominal timeline.
    header.stamp = epoch_ + rclcpp::Duration::from_nanoseconds(index_*period_.nanoseconds());
    ++index_;
    pub_->publish(*cv_bridge::CvImage(header, "bgr8", image).toImageMsg());
  }
  std::string mode_, frame_;
  bof::Color color_{};
  cv::VideoCapture video_;
  std::int64_t index_ = 0;
  rclcpp::Time epoch_{0};
  rclcpp::Duration period_{0,0};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<Source>()); }
  catch (const std::exception& e) { RCLCPP_ERROR(rclcpp::get_logger("source"), "%s", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
