#include "bof_vision/pipeline.hpp"
#include "bof_vision/config.hpp"
#include "diagnostics.hpp"
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <bof_interfaces/msg/armor_candidates.hpp>
#include <bof_interfaces/msg/pixel_track.hpp>
#include <bof_interfaces/msg/armor_poses.hpp>

class Pipeline : public rclcpp::Node {
public:
  Pipeline() : Node("pipeline") {
    // These settings are startup-only. Reject runtime parameter changes instead of
    // reporting values that the active detector/tracker never applies.
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    auto settings = bof::load_config(declare_parameter<std::string>("config_file", "", fixed));
    const auto selected=declare_parameter<std::string>("yolo_name","",fixed);
    if(!selected.empty()) bof::select_yolo(settings,selected);
    auto& config = settings.detector;
    config.enemy_color = bof::parse_color(declare_parameter<std::string>("enemy_color", bof::color_name(config.enemy_color), fixed));
    config.backend = declare_parameter<std::string>("detector_backend", config.backend, fixed);
    config.color_threshold = declare_parameter<int>("color_threshold", config.color_threshold, fixed);
    config.model_path = declare_parameter<std::string>("model_path", config.model_path, fixed);
    config.device = declare_parameter<std::string>("inference_device", config.device, fixed);
    config.confidence_threshold = static_cast<float>(declare_parameter<double>("confidence_threshold", config.confidence_threshold, fixed));
    config.nms_threshold = static_cast<float>(declare_parameter<double>("nms_threshold", config.nms_threshold, fixed));
    settings.track_timeout_s = declare_parameter<double>("track_timeout_s", settings.track_timeout_s, fixed);
    settings.association_gate_px = declare_parameter<double>("association_gate_px", settings.association_gate_px, fixed);
    settings.association.confirm_frames = declare_parameter<int>("track_confirm_frames", settings.association.confirm_frames, fixed);
    settings.association.label_confirm_frames = declare_parameter<int>("track_label_confirm_frames", settings.association.label_confirm_frames, fixed);
    settings.association.ambiguity_px = declare_parameter<double>("association_ambiguity_px", settings.association.ambiguity_px, fixed);
    settings.publish_debug_image = declare_parameter<bool>("publish_debug_image", settings.publish_debug_image, fixed);
    settings.pose.calibration_path = declare_parameter<std::string>("calibration_path", settings.pose.calibration_path, fixed);
    settings.pose.geometry_path = declare_parameter<std::string>("geometry_path", settings.pose.geometry_path, fixed);
    settings.pose.plate_width_m = declare_parameter<double>("plate_width_m", settings.pose.plate_width_m, fixed);
    settings.pose.plate_height_m = declare_parameter<double>("plate_height_m", settings.pose.plate_height_m, fixed);
    settings.pose.label = declare_parameter<int>("pose_label", settings.pose.label, fixed);
    settings.pose.max_reprojection_error_px = declare_parameter<double>("max_reprojection_error_px", settings.pose.max_reprojection_error_px, fixed);
    bof::validate_config(settings);
    const auto snapshot = declare_parameter<std::string>("effective_config_path", "", fixed);
    if (!snapshot.empty()) bof::save_config(settings, snapshot);
    RCLCPP_INFO(get_logger(), "Effective perception configuration:\n%s", bof::config_yaml(settings).c_str());
    timeout_ = settings.track_timeout_s;
    debug_ = settings.publish_debug_image;
    core_ = std::make_unique<bof::PerceptionPipeline>(config,timeout_,settings.association_gate_px,settings.pose,settings.association);
    const auto qos = rclcpp::SensorDataQoS().keep_last(2);
    candidates_ = create_publisher<bof_interfaces::msg::ArmorCandidates>("perception/candidates", qos);
    track_ = create_publisher<bof_interfaces::msg::PixelTrack>("perception/pixel_track", qos);
    poses_ = create_publisher<bof_interfaces::msg::ArmorPoses>("perception/armor_poses", qos);
    image_ = create_publisher<sensor_msgs::msg::Image>("debug/image", qos);
    diagnostics_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics",10);
    generation_sub_=create_subscription<std_msgs::msg::UInt64>("camera/generation",rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::UInt64::ConstSharedPtr value) {
        if(!camera_generation_ || *camera_generation_!=value->data) {
          core_->reset(); active_=false; ++resets_;
          if(frames_) {
            bof_interfaces::msg::PixelTrack empty; empty.header=last_header_; empty.header.stamp=now(); track_->publish(empty);
            bof_interfaces::msg::ArmorPoses poses; poses.header=empty.header; poses_->publish(poses);
          }
        }
        camera_generation_=value->data;
      });
    // Enemy colour from the gimbal link (aim.gimbal_link.use_enemy_color); thresholds stay as configured.
    color_sub_=create_subscription<std_msgs::msg::String>("gimbal/enemy_color",rclcpp::QoS(1).transient_local(),
      [this,config](std_msgs::msg::String::ConstSharedPtr m) {
        try {
          const auto color=bof::parse_color(m->data);
          core_->set_detection_parameters(color,config.color_threshold,config.confidence_threshold,config.nms_threshold);
          core_->reset(); active_=false;
          RCLCPP_WARN(get_logger(),"enemy colour from gimbal link: %s",m->data.c_str());
        } catch(const std::exception& e) {RCLCPP_ERROR(get_logger(),"bad enemy colour: %s",e.what());}
      });
    sub_ = create_subscription<sensor_msgs::msg::Image>("camera/image_raw", qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { process(msg); });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
      if (active_ && std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count() > timeout_) {
        core_->reset(); active_ = false;
        bof_interfaces::msg::PixelTrack empty;
        empty.header = last_header_; empty.header.stamp = now();
        track_->publish(empty);
        bof_interfaces::msg::ArmorPoses poses; poses.header=empty.header; poses_->publish(poses);
      }
    });
    diagnostic_timer_=create_wall_timer(std::chrono::seconds(1),[this] {
      const double silence=frames_ ? std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count() : -1;
      publish_diagnostic(diagnostics_,now(),"bof/perception",silence<0 || silence>timeout_ ? 1 : 0,
        silence<0 ? "waiting_for_image" : silence>timeout_ ? "input_stale" : "processing",{
          {"frames",std::to_string(frames_)},{"errors",std::to_string(errors_)},{"camera_resets",std::to_string(resets_)},
          {"last_processing_ms",std::to_string(process_ms_)},{"header_to_callback_ms",std::to_string(age_ms_)},
          {"input_period_ms",std::to_string(period_ms_)},{"silence_s",std::to_string(silence)},
          {"timestamp_note","header age requires a common clock; not exposure latency"}});
    });
    RCLCPP_INFO(get_logger(), "backend=%s; camera-optical PnP=%s; observation outputs only",config.backend.c_str(),
                settings.pose.calibration_path.empty() ? "off" : "on");
  }
private:
  void process(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    const auto start = std::chrono::steady_clock::now();
    ++frames_; received_=start;
    const auto stamp=rclcpp::Time(msg->header.stamp).nanoseconds();
    age_ms_=(now().nanoseconds()-stamp)*1e-6;
    period_ms_=last_stamp_ ? (stamp-*last_stamp_)*1e-6 : 0;
    last_stamp_=stamp;
    try {
      auto input = cv_bridge::toCvShare(msg, "bgr8");
      auto perception = core_->process({input->image,rclcpp::Time(msg->header.stamp).nanoseconds(),sequence_++,
        msg->header.frame_id,bof::ClockDomain::ros});
      const auto& detections = perception.candidates;
      const auto& tracked = perception.track;
      bof_interfaces::msg::ArmorCandidates result;
      result.header = msg->header;
      for (const auto& d : detections) {
        bof_interfaces::msg::ArmorCandidate c;
        c.geometric_score = d.confidence < 0 ? d.score : -1;
        c.model_confidence = d.confidence;
        c.label = static_cast<int>(d.label);
        c.raw_class=d.raw_class; c.size_hint=static_cast<std::uint8_t>(d.size_hint);
        for (std::size_t i=0; i<4; ++i) { c.corners[i].x=d.corners[i].x; c.corners[i].y=d.corners[i].y; }
        result.candidates.push_back(c);
      }
      candidates_->publish(result);
      bof_interfaces::msg::ArmorPoses pose_message;
      pose_message.header=msg->header;
      for(std::size_t i=0;i<perception.poses.size();++i) {
        const auto& p=perception.poses[i];
        bof_interfaces::msg::ArmorPose observation;
        observation.candidate_index=static_cast<std::uint32_t>(i);
        observation.label=static_cast<int>(detections[i].label);
        observation.valid=p.valid; observation.status=p.status;
        observation.position_m.x=p.position_m[0]; observation.position_m.y=p.position_m[1]; observation.position_m.z=p.position_m[2];
        observation.rotation_vector.x=p.rotation_vector[0]; observation.rotation_vector.y=p.rotation_vector[1]; observation.rotation_vector.z=p.rotation_vector[2];
        observation.distance_m=p.distance_m; observation.reprojection_error_px=p.reprojection_error_px;
        observation.alternative_error_px=p.alternative_error_px; observation.ambiguous=p.ambiguous;
        observation.geometry=p.geometry; observation.geometry_source=p.geometry_source;
        observation.width_m=p.width_m; observation.height_m=p.height_m; observation.image_ratio=p.image_ratio;
        pose_message.poses.push_back(observation);
      }
      poses_->publish(pose_message);
      bof_interfaces::msg::PixelTrack output;
      output.header = msg->header; output.valid = tracked.valid; output.observed = tracked.observed;
      output.track_id = tracked.id; output.u = tracked.center.x; output.v = tracked.center.y;
      output.label = static_cast<int>(tracked.label);
      output.candidate_index = tracked.candidate_index;
      output.observed_label = static_cast<int>(tracked.observed_label);
      output.status = tracked.status; output.association_error_px = tracked.association_error_px;
      track_->publish(output);
      received_ = start; active_ = tracked.id!=0; last_header_ = msg->header;
      if (debug_ && image_->get_subscription_count() > 0) {
        auto canvas = input->image.clone();
        for (const auto& d : detections) for (std::size_t i=0; i<4; ++i)
          cv::line(canvas, d.corners[i], d.corners[(i+1)%4], {0,255,255}, 2);
        if (tracked.valid) cv::circle(canvas, tracked.center, 5, {0,255,0}, -1);
        cv::putText(canvas, "TRACK "+tracked.status+" id="+std::to_string(tracked.id), {10,25}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0,255,255}, 1);
        image_->publish(*cv_bridge::CvImage(msg->header, "bgr8", canvas).toImageMsg());
      }
      double ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
      process_ms_=ms;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "candidates=%zu track=%s processing=%.2fms",
                          detections.size(), tracked.valid ? "observed" : "invalid", ms);
    } catch (const std::exception& e) {
      ++errors_;
      core_->reset(); active_ = false;
      bof_interfaces::msg::PixelTrack invalid; invalid.header = msg->header; track_->publish(invalid);
      bof_interfaces::msg::ArmorPoses empty; empty.header=msg->header; poses_->publish(empty);
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "frame rejected: %s", e.what());
    }
  }
  std::unique_ptr<bof::PerceptionPipeline> core_;
  std::optional<std::uint64_t> camera_generation_;
  std::optional<std::int64_t> last_stamp_;
  std::uint64_t frames_=0,errors_=0,resets_=0;
  double process_ms_=0,age_ms_=0,period_ms_=0;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr generation_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr color_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  std::uint64_t sequence_ = 0;
  double timeout_{}; bool debug_{}, active_ = false;
  std_msgs::msg::Header last_header_;
  std::chrono::steady_clock::time_point received_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<bof_interfaces::msg::ArmorCandidates>::SharedPtr candidates_;
  rclcpp::Publisher<bof_interfaces::msg::PixelTrack>::SharedPtr track_;
  rclcpp::Publisher<bof_interfaces::msg::ArmorPoses>::SharedPtr poses_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<Pipeline>()); }
  catch (const std::exception& e) { RCLCPP_ERROR(rclcpp::get_logger("pipeline"), "%s", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
