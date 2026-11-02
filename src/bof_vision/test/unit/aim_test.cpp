#include "bof_vision/aim.hpp"
#include "bof_vision/aim_engine.hpp"
#include "bof_vision/gimbal_link.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <opencv2/imgproc.hpp>
namespace {
bof::AimConfig config(const std::string& kind="vehicle") {
  bof::AimConfig c;c.enabled=true;c.timestamp_source="simulation";
  c.extrinsics={{0,0,0,1},{0,0,0},{0,0,0}};c.aimer.muzzle_speed_mps=20;
  const bool outpost=kind=="outpost";
  c.targets[6]={outpost?bof::TargetKind::outpost:bof::TargetKind::vehicle,outpost?3:4,0.25};
  c.outpost.speed_radps=0;c.outpost.min_plate_hits=1;return c;
}
// Feed the plate facing the origin of a target spinning at w around (5,0).
bof::SpatialState spin(bof::SpatialTracker& tracker,int plates,double w,int frames,bool outpost,double noise=0) {
  bof::SpatialState result;
  for(int i=0;i<frames;++i) {
    double time=i*0.01, yaw=w*time;
    int plate=0;double angle=10;
    for(int j=0;j<plates;++j) {
      double a=std::remainder(yaw+j*2*M_PI/plates,2*M_PI);
      if(std::abs(a)<std::abs(angle)) {angle=a;plate=j;}
    }
    double height=outpost?0.03*plate:0.03*(plate%2);
    double n=noise*std::sin(i*1.7);
    result=tracker.update(1000000000LL+i*10000000LL,{{6,{5-0.25*std::cos(angle),-0.25*std::sin(angle),0.2+height},angle+n}});
  }
  return result;
}
TEST(Aim, RotatingModelsTrackAcrossAngleWrapAndPlateChanges) {
  for(const auto& kind:{"vehicle","outpost"}) {
    auto c=config(kind);c.tracking.ambiguity_margin=0;
    bof::SpatialTracker tracker(c);
    auto result=spin(tracker,c.targets.at(6).plates,0.8,600,kind==std::string("outpost"));
    ASSERT_TRUE(result.valid)<<kind<<":"<<result.status;
    EXPECT_NEAR(result.x[0],5,0.06);EXPECT_NEAR(result.x[2],0,0.06);EXPECT_NEAR(result.x[7],0.8,0.12);
    auto lost=tracker.update(8000000000LL,{});EXPECT_FALSE(lost.valid);
  }
}
TEST(Aim, OutpostVotePinsRuleSpeedAndRecognisesStillOutpost) {
  auto c=config("outpost");c.tracking.ambiguity_margin=0;c.outpost.speed_radps=2.513274;
  for(double w:{2.513274,-2.513274}) {
    bof::SpatialTracker tracker(c);
    auto result=spin(tracker,3,w,300,true,0.01);
    ASSERT_TRUE(result.valid)<<result.status;
    EXPECT_EQ(result.spin,w>0?bof::Spin::counter_clockwise:bof::Spin::clockwise);
    EXPECT_DOUBLE_EQ(result.x[7],w);
  }
  bof::SpatialTracker still(c);
  auto result=spin(still,3,0,100,true,0.01);
  EXPECT_EQ(result.spin,bof::Spin::stopped);EXPECT_DOUBLE_EQ(result.x[7],0);
  std::vector<double> jumps{0,0.1,0.2,0.3,0.4-2*M_PI/3,0.5-2*M_PI/3,0.6-2*M_PI/3,0.7-2*M_PI/3};
  std::vector<double> times{0,0.04,0.08,0.12,0.16,0.2,0.24,0.28};
  EXPECT_EQ(bof::vote_direction(jumps,times,1,0.6,0.4),bof::Spin::counter_clockwise);
}
TEST(Aim, OutpostStoppedPolicyOverridesVote) {
  auto c=config("outpost");bof::SpatialTracker tracker(c);
  bof::TargetPolicy policy;policy.outpost_stopped=true;tracker.set_policy(policy);
  auto result=spin(tracker,3,0.8,50,true);
  EXPECT_EQ(result.spin,bof::Spin::stopped);EXPECT_DOUBLE_EQ(result.x[7],0);
}
TEST(Aim, TrajectoryIntersectsPredictedPlateAndRejectsUnsynchronizedTime) {
  auto c=config();bof::SpatialTracker tracker(c);bof::SpatialState state;
  for(int i=0;i<20;++i) state=tracker.update(1000000000LL+i*10000000LL,{{6,{4.75,0,0.2},0}});
  auto result=bof::solve_aim(state,c,0.02,Eigen::Vector3d::Zero(),20);ASSERT_TRUE(result.valid)<<result.status;
  const double t=result.flight_time_s;
  Eigen::Vector3d impact(20*std::cos(result.pitch_rad)*std::cos(result.yaw_rad)*t,
    20*std::cos(result.pitch_rad)*std::sin(result.yaw_rad)*t,
    20*std::sin(result.pitch_rad)*t-c.aimer.gravity_mps2*t*t/2);
  EXPECT_LT((impact-result.point).norm(),1e-3);
  c.timestamp_source="host_receive";EXPECT_FALSE(bof::solve_aim(state,c,0.02,Eigen::Vector3d::Zero(),20).valid);
  c.timestamp_source="simulation";EXPECT_FALSE(bof::solve_aim(state,c,0.3,Eigen::Vector3d::Zero(),20).valid);
}
TEST(Aim, SpinningTargetUsesComingPlateWindow) {
  auto c=config();c.tracking.ambiguity_margin=0;bof::SpatialTracker tracker(c);
  auto state=spin(tracker,4,6.0,400,false);
  ASSERT_TRUE(state.valid)<<state.status;
  int windows=0,gaps=0;
  for(double age=0;age<0.2;age+=0.005) {
    auto solution=bof::solve_aim(state,c,age,Eigen::Vector3d::Zero(),20);
    ASSERT_TRUE(solution.valid)<<solution.status;
    if(!solution.in_window) {++gaps; EXPECT_LT(solution.facing_rad,-c.aimer.coming_angle_deg*M_PI/180); continue;}
    ++windows;
    // omega>0: the plate is still coming (negative) or less than leaving_angle past the centre.
    EXPECT_LT(solution.facing_rad,c.aimer.leaving_angle_deg*M_PI/180+1e-6);
    EXPECT_GT(solution.facing_rad,-c.aimer.coming_angle_deg*M_PI/180-1e-6);
  }
  EXPECT_GT(windows,0);EXPECT_GT(gaps,0);
}
TEST(Aim, ShooterNeedsSteadyCommandAndGimbalOnPlate) {
  auto c=config();c.shooter.auto_fire=true;bof::Shooter shooter(c.shooter);
  bof::AimSolution s;s.valid=true;s.plate=0;s.yaw_rad=0.1;s.pitch_rad=0.02;s.distance_m=5;
  EXPECT_FALSE(shooter.decide(s,1,{0.1,0.02},true).fire);   // first command
  EXPECT_TRUE(shooter.decide(s,1,{0.1,0.02},true).fire);
  EXPECT_EQ(shooter.decide(s,1,{0.15,0.02},true).status,"gimbal_not_on_target");
  EXPECT_EQ(shooter.decide(s,1,{0.1,0.02},false).status,"fire_not_enabled");
  s.plate=1;EXPECT_EQ(shooter.decide(s,1,{0.1,0.02},true).status,"command_changed");
}
TEST(Aim, PolicyExcludesLocksAndSwitchesToHigherPriority) {
  auto c=config();c.targets[1]={bof::TargetKind::vehicle,4,0.25};c.selection.switch_frames=3;
  bof::SpatialTracker tracker(c);
  bof::TargetPolicy policy;policy.excluded={1};tracker.set_policy(policy);
  std::vector<bof::SpatialObservation> both{{6,{4.75,0,0.2},0},{1,{3.75,1,0.2},0}};
  auto s=tracker.update(1000000000LL,both);EXPECT_EQ(s.label,6);
  tracker.set_policy({});
  for(int i=1;i<3;++i) s=tracker.update(1000000000LL+i*10000000LL,both);
  EXPECT_EQ(s.label,6);
  s=tracker.update(1030000000LL,both);EXPECT_EQ(s.label,1);   // hero ranks first after 3 frames
  policy={};policy.lock_label=6;tracker.set_policy(policy);
  s=tracker.update(1040000000LL,both);EXPECT_EQ(s.label,6);   // hero released, locked label re-acquired
}
TEST(Aim, SpFramesRoundTripWithCrc) {
  bof::sp::VisionToGimbal command{};command.mode=2;command.yaw=1.5F;
  auto bytes=bof::encode(command);ASSERT_EQ(bytes.size(),sizeof(command));
  const std::uint16_t crc=static_cast<std::uint16_t>(bytes[bytes.size()-2]|(bytes.back()<<8));
  EXPECT_EQ(bof::crc16(bytes.data(),bytes.size()-2),crc);
  bof::sp::GimbalToVision feedback{};feedback.mode=1;feedback.q[0]=1;feedback.bullet_speed=23;
  feedback.crc16=bof::crc16(reinterpret_cast<const std::uint8_t*>(&feedback),sizeof(feedback)-2);
  auto* raw=reinterpret_cast<std::uint8_t*>(&feedback);
  auto decoded=bof::decode(raw,sizeof(feedback));ASSERT_TRUE(decoded);EXPECT_FLOAT_EQ(decoded->bullet_speed,23);
  raw[5]^=1;EXPECT_FALSE(bof::decode(raw,sizeof(feedback)));
}
TEST(Aim, MissingPhysicalInputsFailAndConfigurationRoundTrips) {
  auto c=config();auto node=bof::aim_config_yaml(c);bof::AimConfig copy;bof::load_aim_config(node,copy);
  EXPECT_NO_THROW(bof::validate_aim_config(copy));EXPECT_EQ(copy.targets.at(6).plates,4);
  copy.extrinsics.camera_to_gimbal_q_xyzw.clear();EXPECT_THROW(bof::validate_aim_config(copy),std::invalid_argument);
  node["surprise"]=1;EXPECT_THROW(bof::load_aim_config(node,copy),std::invalid_argument);
  auto latency=config();latency.timestamp_source="host_compensated";
  EXPECT_THROW(bof::validate_aim_config(latency),std::invalid_argument);
}
// Engine: 1 s of a static vehicle 4.75 m ahead, camera = gimbal frame rotated to optical axes.
std::unique_ptr<bof::AimEngine> engine_with_target(bof::AimConfig c) {
  c.extrinsics.camera_to_gimbal_q_xyzw={0.5,-0.5,0.5,-0.5};
  auto engine=std::make_unique<bof::AimEngine>(c);
  for(int i=0;i<=20;++i) engine->push_orientation(1000000000LL+i*10000000LL,Eigen::Quaterniond::Identity());
  for(int i=0;i<20;++i) {
    const std::int64_t t=1000000000LL+i*10000000LL;
    // Gimbal (x fwd, y left, z up) -> optical (x right, y down, z fwd); plate normal points away from the camera.
    engine->enqueue({t,{{6,{0,-0.2,4.75},{0,0,0}}}},t+1000000);
  }
  return engine;
}
TEST(AimEngine, CommandFollowsDirectiveAndFallsBackWhenStale) {
  auto c=config();c.shooter.auto_fire=true;c.decision.enabled=true;c.decision.require_directive=true;c.tracking.timeout_s=1;
  auto engine=engine_with_target(c);auto& e=*engine;
  const std::int64_t t=1200000000LL;
  auto out=e.step(t);
  ASSERT_TRUE(out.solution.valid)<<out.solution.status;
  EXPECT_FALSE(out.command.control);EXPECT_EQ(out.fire.status,"aim_not_enabled");
  bof::Directive d;d.fire_enable=true;e.set_directive(d,t);
  out=e.step(t+1000000);EXPECT_TRUE(out.command.control);
  EXPECT_NEAR(out.command.yaw,0,1e-3);EXPECT_GT(out.command.pitch,0);
  e.push_orientation(t+600000000LL,Eigen::Quaterniond::Identity());
  out=e.step(t+600000000LL);EXPECT_FALSE(out.command.control);   // directive older than 0.5 s
}
TEST(AimEngine, MeasuredBulletSpeedOnlyInPlausibleRange) {
  auto c=config();c.tracking.timeout_s=1;c.gimbal_link.use_bullet_speed=true;auto engine=engine_with_target(c);auto& e=*engine;
  bof::GimbalFeedback f;f.stamp_ns=1190000000LL;f.bullet_speed=25;e.push_feedback(f);
  EXPECT_DOUBLE_EQ(e.step(1200000000LL).bullet_speed_mps,25);
  f.stamp_ns=1195000000LL;f.bullet_speed=5;e.push_feedback(f);
  EXPECT_DOUBLE_EQ(e.step(1200000000LL).bullet_speed_mps,20);
}
TEST(AimEngine, StaleImuAndInvalidationStopTheCommand) {
  auto c=config();c.tracking.timeout_s=1;auto engine=engine_with_target(c);auto& e=*engine;
  EXPECT_EQ(e.step(1300000000LL).solution.status,"imu_stale");
  e.invalidate("camera_reconnected");
  EXPECT_FALSE(e.step(1200000000LL).command.control);
  EXPECT_EQ(e.output().state.status,"camera_reconnected");
}
TEST(AimEngine, OverlayProjectsMatchedPlateWhereTheCameraSeesIt) {
  auto c=config();c.extrinsics.camera_to_gimbal_q_xyzw={0.5,-0.5,0.5,-0.5};
  bof::SpatialTracker tracker(c);bof::SpatialState state;
  for(int i=0;i<10;++i) state=tracker.update(1000000000LL+i*10000000LL,{{6,{4.75,0,0.2},0}});
  ASSERT_TRUE(state.valid);
  bof::CameraCalibration camera{{640,480},(cv::Mat_<double>(3,3)<<500,0,320,0,500,240,0,0,1),cv::Mat::zeros(1,5,CV_64F)};
  cv::Mat image(480,640,CV_8UC3,cv::Scalar(0,0,0));
  bof::TrackUpdate u{state,{},0,Eigen::Quaterniond::Identity()};
  ASSERT_TRUE(bof::draw_aim_overlay(image,camera,c,u,{}));
  // Plate 0 centre (4.75, 0, 0.2) -> u = 320, v = 240 - 500*0.2/4.75 ~= 219; the drawn rectangle's
  // bottom edge is 0.0275 m lower -> v ~= 222. The plate number label sits above, so check the bottom.
  cv::Mat green; cv::inRange(image,cv::Scalar(0,250,0),cv::Scalar(10,255,10),green);
  const cv::Rect box=cv::boundingRect(green(cv::Rect(0,100,640,380)));
  EXPECT_NEAR(box.x+box.width/2.0,320,3); EXPECT_NEAR(100+box.y+box.height,222+1,2);
}
TEST(Aim, CboardFramesFollowFirmwareConventions) {
  auto p=bof::cboard_protocol();
  bof::GimbalCommand c;c.control=true;c.fire=true;c.yaw=0.3;c.pitch=0.1;
  auto bytes=p.encode(c);
  ASSERT_EQ(bytes.size(),sizeof(bof::cboard::VisionToGimbal));
  EXPECT_EQ(bytes[0],0x50);EXPECT_EQ(bytes[1],2);
  float pitch;std::memcpy(&pitch,bytes.data()+2+12,4);EXPECT_FLOAT_EQ(pitch,0.1F);   // elevation, up positive
  EXPECT_EQ(bof::crc16(bytes.data(),bytes.size()-2),static_cast<std::uint16_t>(bytes[bytes.size()-2]|(bytes.back()<<8)));
  bof::cboard::GimbalToVision f{};f.head=0x53;f.mode=1;f.enemy_color=1;f.q[0]=1;f.pitch=0.2F;f.bullet_speed=22;
  f.crc16=bof::crc16(reinterpret_cast<const std::uint8_t*>(&f),sizeof(f)-2);
  ASSERT_EQ(p.feedback_size,43U);
  auto decoded=p.decode(reinterpret_cast<const std::uint8_t*>(&f));
  ASSERT_TRUE(decoded);EXPECT_EQ(decoded->enemy_color,1);EXPECT_DOUBLE_EQ(decoded->pitch,-0.2F);   // INS.Roll is nose down
  f.mode=0;EXPECT_FALSE(p.decode(reinterpret_cast<const std::uint8_t*>(&f)));
}
}
