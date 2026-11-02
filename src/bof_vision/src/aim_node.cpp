#include "bof_vision/aim_engine.hpp"
#include "bof_vision/config.hpp"
#include "diagnostics.hpp"
#include <bof_interfaces/msg/aim_directive.hpp>
#include <bof_interfaces/msg/aim_solution.hpp>
#include <bof_interfaces/msg/armor_poses.hpp>
#include <bof_interfaces/msg/auto_aim_target.hpp>
#include <bof_interfaces/msg/gimbal_command.hpp>
#include <bof_interfaces/msg/gimbal_state.hpp>
#include <bof_interfaces/msg/spatial_target.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <deque>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

// Message conversion and threads only; synchronisation, tracking and fire control live in
// bof::AimEngine. The command loop (Tongji CommandGener pattern) runs at aim.command_rate_hz.
class AimNode final:public rclcpp::Node {
public:
  AimNode():Node("aim") {
    rcl_interfaces::msg::ParameterDescriptor d; d.read_only=true;
    const auto path=declare_parameter<std::string>("config_file","",d);
    const auto settings=bof::load_config(path);
    auto config=settings.aim;
    bof::validate_aim_config(config);
    if(!config.enabled) throw std::invalid_argument("aim.enabled is false; supply measured configuration before enabling aim_node");
    engine_=std::make_unique<bof::AimEngine>(config);
    const auto& c=engine_->config();
    target_pub_=create_publisher<bof_interfaces::msg::SpatialTarget>("tracking/spatial_target",10);
    solution_pub_=create_publisher<bof_interfaces::msg::AimSolution>("aim/solution",10);
    if(c.decision.enabled) decision_pub_=create_publisher<bof_interfaces::msg::AutoAimTarget>("auto_aim/target",10);
    diagnostics_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics",10);
    if(c.publish_gimbal_io && c.gimbal_link.imu_source=="serial") {
      // Serial feedback and commands as topics, so rosbag can record image+attitude for replay/calibration.
      imu_pub_=create_publisher<sensor_msgs::msg::Imu>("gimbal/imu",rclcpp::SensorDataQoS().keep_last(200));
      state_pub_=create_publisher<bof_interfaces::msg::GimbalState>("gimbal/state",rclcpp::SensorDataQoS().keep_last(200));
      command_pub_=create_publisher<bof_interfaces::msg::GimbalCommand>("gimbal/command",rclcpp::SensorDataQoS().keep_last(200));
    }
    if(c.gimbal_link.use_enemy_color)
      color_pub_=create_publisher<std_msgs::msg::String>("gimbal/enemy_color",rclcpp::QoS(1).transient_local());
    gimbal_=bof::make_gimbal_link(c.gimbal_link,[this]{return now().nanoseconds();},
      [this](const bof::GimbalFeedback& f){engine_->push_feedback(f); publish_feedback(f);});
    if(!gimbal_) {
      imu_sub_=create_subscription<sensor_msgs::msg::Imu>("gimbal/imu",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::Imu::ConstSharedPtr m){
        if(m->header.frame_id!=engine_->config().imu_frame || m->orientation_covariance[0]<0) {++bad_imu_; return;}
        engine_->push_orientation(rclcpp::Time(m->header.stamp).nanoseconds(),
          Eigen::Quaterniond(m->orientation.w,m->orientation.x,m->orientation.y,m->orientation.z));
        publish_targets(engine_->drain(now().nanoseconds()));
      });
    }
    pose_sub_=create_subscription<bof_interfaces::msg::ArmorPoses>("perception/armor_poses",rclcpp::SensorDataQoS(),[this](bof_interfaces::msg::ArmorPoses::ConstSharedPtr m){
      if(m->header.frame_id!=engine_->config().camera_frame) {publish_target(engine_->invalidate("camera_frame_mismatch"));return;}
      bof::PoseFrame frame; frame.stamp_ns=rclcpp::Time(m->header.stamp).nanoseconds();
      for(const auto& p:m->poses) {
        if(!p.valid || p.ambiguous) continue;
        frame.poses.push_back({p.label,{p.position_m.x,p.position_m.y,p.position_m.z},
                               {p.rotation_vector.x,p.rotation_vector.y,p.rotation_vector.z}});
      }
      publish_targets(engine_->enqueue(std::move(frame),now().nanoseconds()));
    });
    if(c.decision.enabled) directive_sub_=create_subscription<bof_interfaces::msg::AimDirective>("sentry/aim_directive",10,[this](bof_interfaces::msg::AimDirective::ConstSharedPtr m){
      if(m->lock_label<-1 || m->lock_label>7 || m->priority_label<-1 || m->priority_label>7) {++bad_directives_; return;}
      bof::Directive directive; directive.aim_enable=m->aim_enable; directive.fire_enable=m->fire_enable;
      for(int label:m->excluded_labels) if(label>=0 && label<=7) directive.policy.excluded.insert(label);
      directive.policy.lock_label=m->lock_label; directive.policy.priority_label=m->priority_label;
      directive.policy.outpost_stopped=m->outpost_stopped;
      engine_->set_directive(directive,now().nanoseconds());
    });
    generation_sub_=create_subscription<std_msgs::msg::UInt64>("camera/generation",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::UInt64::ConstSharedPtr m){
      if(generation_ && generation_!=m->data) publish_target(engine_->invalidate("camera_reconnected"));
      generation_=m->data;
    });
    if(c.debug_image_rate_hz>0 && !settings.pose.calibration_path.empty()) {
      // Overlay on the pipeline debug image (same header stamp as the poses), throttled; off in matches.
      camera_=bof::load_calibration(settings.pose.calibration_path);
      debug_pub_=create_publisher<sensor_msgs::msg::Image>("aim/debug_image",rclcpp::SensorDataQoS().keep_last(2));
      debug_sub_=create_subscription<sensor_msgs::msg::Image>("debug/image",rclcpp::SensorDataQoS().keep_last(2),
        [this](sensor_msgs::msg::Image::ConstSharedPtr m){
          const auto t=std::chrono::steady_clock::now();
          if(t-last_debug_<std::chrono::duration<double>(1/engine_->config().debug_image_rate_hz)) return;
          if(debug_pub_->get_subscription_count()==0) return;
          last_debug_=t; debug_images_.push_back(m); if(debug_images_.size()>4) debug_images_.pop_front();
          publish_debug();
        });
    }
    drain_timer_=create_wall_timer(std::chrono::milliseconds(5),[this]{
      const auto t=now().nanoseconds();
      publish_targets(engine_->drain(t));
      if(auto expired=engine_->expire(t)) publish_target(*expired);
    });
    publish_timer_=create_wall_timer(std::chrono::duration<double>(1/c.publish_rate_hz),[this]{publish_solution();});
    diagnostic_timer_=create_wall_timer(std::chrono::seconds(1),[this]{
      const auto out=engine_->output(); const auto counters=engine_->counters(); const auto& c=engine_->config();
      publish_diagnostic(diagnostics_,now(),"bof/aim",out.solution.valid?0:1,out.solution.status,
        {{"sync_drops",std::to_string(counters.sync_drops)},{"rejected_imu",std::to_string(counters.rejected_imu+bad_imu_)},
         {"rejected_directives",std::to_string(bad_directives_)},{"fire_status",out.fire.status},
         {"timestamp_source",c.timestamp_source},{"origin",c.origin},{"imu_source",c.gimbal_link.imu_source},
         {"gimbal_link",gimbal_?(gimbal_->connected()?"connected":"disconnected"):"none"},
         {"gimbal_frames",gimbal_?std::to_string(gimbal_->frames()):"0"},
         {"gimbal_crc_errors",gimbal_?std::to_string(gimbal_->crc_errors()):"0"},
         {"command_loop_overruns",std::to_string(overruns_.load())}});
    });
    command_thread_=std::thread([this]{command_loop();});
  }
  ~AimNode() override {
    quit_=true;
    if(command_thread_.joinable()) command_thread_.join();
    if(gimbal_) gimbal_->send({});
  }
private:
  void command_loop() {
    const auto period=std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1/engine_->config().command_rate_hz));
    auto next=std::chrono::steady_clock::now();
    while(!quit_ && rclcpp::ok()) {
      next+=period;
      const auto t=now();
      const auto out=engine_->step(t.nanoseconds());
      if(gimbal_) gimbal_->send(out.command);
      if(command_pub_ && gimbal_) {
        bof_interfaces::msg::GimbalCommand m; m.header.stamp=t; m.header.frame_id=engine_->config().reference_frame;
        m.control=out.command.control; m.fire=out.command.fire; m.yaw=out.command.yaw; m.pitch=out.command.pitch;
        m.track_id=out.state.id; m.plate=out.solution.plate; command_pub_->publish(m);
      }
      if(std::chrono::steady_clock::now()>next) {++overruns_; next=std::chrono::steady_clock::now();}
      std::this_thread::sleep_until(next);
    }
  }
  void publish_feedback(const bof::GimbalFeedback& f) {
    if(color_pub_ && f.enemy_color>=0 && f.enemy_color!=last_color_.exchange(f.enemy_color)) {
      std_msgs::msg::String m; m.data=f.enemy_color==0?"red":"blue"; color_pub_->publish(m);
    }
    if(!imu_pub_) return;
    sensor_msgs::msg::Imu imu; imu.header.stamp=rclcpp::Time(f.stamp_ns); imu.header.frame_id=engine_->config().imu_frame;
    imu.orientation.w=f.orientation.w(); imu.orientation.x=f.orientation.x();
    imu.orientation.y=f.orientation.y(); imu.orientation.z=f.orientation.z();
    imu.angular_velocity_covariance[0]=imu.linear_acceleration_covariance[0]=-1;   // not provided by the link
    imu_pub_->publish(imu);
    bof_interfaces::msg::GimbalState s; s.header=imu.header; s.mode=static_cast<uint8_t>(f.mode);
    s.yaw=f.yaw; s.yaw_vel=f.yaw_vel; s.pitch=f.pitch; s.pitch_vel=f.pitch_vel;
    s.bullet_speed_mps=f.bullet_speed; s.bullet_count=static_cast<uint32_t>(f.bullet_count);
    state_pub_->publish(s);
  }
  void publish_targets(const std::vector<bof::TrackUpdate>& updates) {
    for(const auto& u:updates) {
      publish_target(u.state);
      if(debug_pub_ && u.orientation) {updates_.push_back(u); if(updates_.size()>30) updates_.pop_front();}
    }
    if(debug_pub_ && !updates.empty()) publish_debug();
  }
  // Draw each stored image once its tracker update (same stamp) is available.
  void publish_debug() {
    for(auto it=debug_images_.begin();it!=debug_images_.end();) {
      const auto stamp=rclcpp::Time((*it)->header.stamp).nanoseconds();
      auto u=std::find_if(updates_.begin(),updates_.end(),[&](const auto& x){return x.frame_stamp_ns==stamp;});
      if(u==updates_.end()) {++it; continue;}
      auto image=cv_bridge::toCvCopy(*it,"bgr8");
      if(image->image.size()==camera_.image_size) bof::draw_aim_overlay(image->image,camera_,engine_->config(),*u,engine_->output());
      debug_pub_->publish(*image->toImageMsg());
      it=debug_images_.erase(it);
    }
  }
  void publish_target(const bof::SpatialState& s) {
    bof_interfaces::msg::SpatialTarget m;
    m.header.stamp=s.stamp_ns?rclcpp::Time(s.stamp_ns):now();m.header.frame_id=engine_->config().reference_frame;
    m.valid=s.valid;m.status=s.status;m.track_id=s.id;m.label=s.label;
    m.model=bof::kind_name(s.model.kind);m.plates=s.model.plates;m.matched_plate=s.matched_plate;m.radius_m=s.model.radius_m;
    m.spin=bof::spin_name(s.spin);
    if(s.x.size()) m.state.assign(s.x.data(),s.x.data()+s.x.size());
    target_pub_->publish(m);
  }
  void publish_solution() {
    const auto out=engine_->output();
    const auto t=now(); const auto& frame=engine_->config().reference_frame;
    const auto& s=out.state; const auto& solution=out.solution;
    bof_interfaces::msg::AimSolution m;
    m.header.stamp=t;m.header.frame_id=frame;
    if(s.stamp_ns) m.observation_stamp=rclcpp::Time(s.stamp_ns);
    m.valid=solution.valid;m.status=solution.status;m.track_id=s.id;m.label=s.label;m.plate=solution.plate;
    m.yaw_rad=solution.yaw_rad;m.pitch_rad=solution.pitch_rad;m.flight_time_s=solution.flight_time_s;m.observation_age_s=out.age_s;
    m.predicted_point_m.x=solution.point.x();m.predicted_point_m.y=solution.point.y();m.predicted_point_m.z=solution.point.z();
    m.fire=out.fire.fire;m.fire_status=out.fire.status;
    solution_pub_->publish(m);
    bof_interfaces::msg::AutoAimTarget target;
    target.header.stamp=s.stamp_ns?rclcpp::Time(s.stamp_ns):t;target.header.frame_id=frame;
    target.tracking_state=s.status;target.valid=s.valid;target.track_id=s.id;target.label=s.label;
    target.target_type=s.x.size()?bof::kind_name(s.model.kind):"";target.armor_num=s.x.size()?s.model.plates:0;
    if(s.x.size()) {
      target.position.x=s.x[0];target.position.y=s.x[2];target.position.z=s.x[4];
      target.velocity.x=s.x[1];target.velocity.y=s.x[3];target.velocity.z=s.x[5];
      target.yaw=s.x[6];target.v_yaw=s.x[7];
      target.bearing_rad=std::atan2(s.x[2],s.x[0]);target.distance_m=std::sqrt(s.x[0]*s.x[0]+s.x[2]*s.x[2]+s.x[4]*s.x[4]);
    }
    target.spin=bof::spin_name(s.spin);target.locked=out.locked;target.aiming=out.aiming;
    target.firing=out.fire.fire;target.fire_status=out.fire.status;target.bullet_speed_mps=out.bullet_speed_mps;
    if(decision_pub_) decision_pub_->publish(target);
  }
  std::unique_ptr<bof::AimEngine> engine_;
  std::unique_ptr<bof::GimbalIO> gimbal_;
  bof::CameraCalibration camera_;
  std::deque<bof::TrackUpdate> updates_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> debug_images_;
  std::chrono::steady_clock::time_point last_debug_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_sub_;
  uint64_t generation_=0;
  std::atomic<uint64_t> bad_imu_{0},bad_directives_{0},overruns_{0};
  std::atomic<bool> quit_{false};
  std::thread command_thread_;
  rclcpp::Publisher<bof_interfaces::msg::SpatialTarget>::SharedPtr target_pub_;
  rclcpp::Publisher<bof_interfaces::msg::AimSolution>::SharedPtr solution_pub_;
  rclcpp::Publisher<bof_interfaces::msg::AutoAimTarget>::SharedPtr decision_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_pub_;
  std::atomic<int> last_color_{-1};
  rclcpp::Publisher<bof_interfaces::msg::GimbalState>::SharedPtr state_pub_;
  rclcpp::Publisher<bof_interfaces::msg::GimbalCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<bof_interfaces::msg::ArmorPoses>::SharedPtr pose_sub_;
  rclcpp::Subscription<bof_interfaces::msg::AimDirective>::SharedPtr directive_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr generation_sub_;
  rclcpp::TimerBase::SharedPtr drain_timer_,publish_timer_,diagnostic_timer_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<AimNode>());}
  catch(const std::exception& e) {RCLCPP_ERROR(rclcpp::get_logger("aim"),"%s",e.what());if(rclcpp::ok())rclcpp::shutdown();return 1;}
  if(rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
