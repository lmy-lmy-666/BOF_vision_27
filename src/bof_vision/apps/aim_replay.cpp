// Offline replay and latency analysis of a recorded rosbag through bof::AimEngine.
// Inputs: /perception/armor_poses, /gimbal/imu, optional /gimbal/state, /gimbal/command, /sentry/aim_directive.
#include "bof_vision/aim_engine.hpp"
#include "bof_vision/config.hpp"
#include <bof_interfaces/msg/aim_directive.hpp>
#include <bof_interfaces/msg/armor_poses.hpp>
#include <bof_interfaces/msg/gimbal_command.hpp>
#include <bof_interfaces/msg/gimbal_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <variant>

namespace {
constexpr double pi=3.14159265358979323846;
using Poses=bof_interfaces::msg::ArmorPoses;
using Imu=sensor_msgs::msg::Imu;
using State=bof_interfaces::msg::GimbalState;
using Command=bof_interfaces::msg::GimbalCommand;
using DirectiveMsg=bof_interfaces::msg::AimDirective;
struct Event {std::int64_t recv_ns; std::variant<Poses,Imu,State,Command,DirectiveMsg> message;};
std::int64_t stamp(const builtin_interfaces::msg::Time& t) {return rclcpp::Time(t).nanoseconds();}
template<class T> T decode(const rosbag2_storage::SerializedBagMessage& m) {
  static rclcpp::Serialization<T> serialization; T message;
  rclcpp::SerializedMessage serialized(*m.serialized_data);
  serialization.deserialize_message(&serialized,&message);
  return message;
}
std::vector<Event> read_bag(const std::string& path) {
  rosbag2_cpp::Reader reader; reader.open(path);
  std::vector<Event> events;
  while(reader.has_next()) {
    const auto m=reader.read_next();
    const auto& topic=m->topic_name;
    if(topic=="/perception/armor_poses") events.push_back({m->recv_timestamp,decode<Poses>(*m)});
    else if(topic=="/gimbal/imu") events.push_back({m->recv_timestamp,decode<Imu>(*m)});
    else if(topic=="/gimbal/state") events.push_back({m->recv_timestamp,decode<State>(*m)});
    else if(topic=="/gimbal/command") events.push_back({m->recv_timestamp,decode<Command>(*m)});
    else if(topic=="/sentry/aim_directive") events.push_back({m->recv_timestamp,decode<DirectiveMsg>(*m)});
  }
  std::stable_sort(events.begin(),events.end(),[](const Event& a,const Event& b){return a.recv_ns<b.recv_ns;});
  return events;
}
bof::PoseFrame frame_of(const Poses& m) {
  bof::PoseFrame f; f.stamp_ns=stamp(m.header.stamp);
  for(const auto& p:m.poses) if(p.valid && !p.ambiguous)
    f.poses.push_back({p.label,{p.position_m.x,p.position_m.y,p.position_m.z},{p.rotation_vector.x,p.rotation_vector.y,p.rotation_vector.z}});
  return f;
}
Eigen::Quaterniond quaternion(const Imu& m) {return {m.orientation.w,m.orientation.x,m.orientation.y,m.orientation.z};}
Eigen::Vector2d pointing(const Eigen::Quaterniond& q) {
  const Eigen::Vector3d f=q*Eigen::Vector3d::UnitX();
  return {std::atan2(f.y(),f.x()),std::atan2(f.z(),std::hypot(f.x(),f.y()))};
}
struct Stat {
  int n=0; double sum=0, sq=0;
  void add(double v) {++n; sum+=v; sq+=v*v;}
  double mean() const {return n?sum/n:0;}
  double sd() const {return n>1?std::sqrt(std::max(0.0,sq/n-mean()*mean())):0;}
};
double median(std::vector<double> v) {
  if(v.empty()) return NAN;
  std::nth_element(v.begin(),v.begin()+v.size()/2,v.end()); return v[v.size()/2];
}
// Engine replay: same command-loop timing as aim_node, bag receive time as "now".
void replay(const std::vector<Event>& events,const bof::AimConfig& config,const std::string& out,double rate_hz) {
  bof::AimEngine engine(config);
  std::ofstream tracks(out+"/tracks.csv"), commands(out+"/commands.csv");
  tracks << "t_s,track_id,label,kind,status,valid,spin,plate,obs_count,x,vx,y,vy,z,vz,yaw,omega,s8,s9,s10,res_x,res_y,res_z\n";
  commands << "t_s,track_id,valid,status,plate,in_window,yaw,pitch,gimbal_yaw,gimbal_pitch,fire,fire_status,bullet_speed\n";
  std::map<int,Stat> residual_z[2]; Stat residual_xy;   // per plate, [0] vehicle [1] outpost
  std::map<std::string,int> fire_status; std::map<std::uint64_t,double> spin_time; std::map<std::uint64_t,std::string> spin_value;
  std::map<std::uint64_t,bof::SpatialState> previous;
  int fire=0, steps=0, updates=0; std::int64_t first=events.empty()?0:events.front().recv_ns;
  const std::int64_t period=static_cast<std::int64_t>(1e9/rate_hz);
  std::int64_t next_tick=first; Eigen::Quaterniond gimbal=Eigen::Quaterniond::Identity();
  auto log_updates=[&](const std::vector<bof::TrackUpdate>& list) {
    for(const auto& u:list) {
      const auto& s=u.state; ++updates;
      tracks << std::fixed << std::setprecision(4) << (s.stamp_ns-first)*1e-9 << ',' << s.id << ',' << s.label << ','
             << bof::kind_name(s.model.kind) << ',' << s.status << ',' << s.valid << ',' << bof::spin_name(s.spin) << ','
             << s.matched_plate << ',' << u.observations.size();
      for(int i=0;i<11;++i) tracks << ',' << (i<s.x.size()?s.x[i]:NAN);
      Eigen::Vector3d residual=Eigen::Vector3d::Constant(NAN);
      auto prev=previous.find(s.id);
      // A-priori residual: the plate predicted from the previous update against the matched observation.
      if(prev!=previous.end() && s.matched_plate>=0 && prev->second.x.size() && s.stamp_ns>prev->second.stamp_ns) {
        const auto p=bof::predicted_plate(prev->second,s.matched_plate,(s.stamp_ns-prev->second.stamp_ns)*1e-9);
        double best=1e9;
        for(const auto& o:u.observations) if(o.label==s.label && (o.position-p.head<3>()).norm()<best) {
          best=(o.position-p.head<3>()).norm(); residual=o.position-p.head<3>();
        }
        if(residual.allFinite() && s.valid) {
          residual_z[s.model.kind==bof::TargetKind::outpost][s.matched_plate].add(residual.z());
          residual_xy.add(residual.head<2>().norm());
        }
      }
      tracks << ',' << residual.x() << ',' << residual.y() << ',' << residual.z() << '\n';
      if(s.id && s.spin!=bof::Spin::unknown && !spin_time.count(s.id)) {spin_time[s.id]=(s.stamp_ns-first)*1e-9; spin_value[s.id]=bof::spin_name(s.spin);}
      if(s.id) previous[s.id]=s;
    }
  };
  auto tick=[&](std::int64_t t) {
    log_updates(engine.drain(t));
    if(auto expired=engine.expire(t)) log_updates({{*expired,{},0,std::nullopt}});
    const auto o=engine.step(t); ++steps;
    if(o.fire.fire) ++fire;
    if(o.solution.valid) ++fire_status[o.fire.status];
    if(steps%std::max(1,static_cast<int>(rate_hz/100))==0) {
      const auto g=pointing(gimbal);
      commands << std::fixed << std::setprecision(5) << (t-first)*1e-9 << ',' << o.state.id << ',' << o.solution.valid << ','
               << o.solution.status << ',' << o.solution.plate << ',' << o.solution.in_window << ',' << o.solution.yaw_rad << ','
               << o.solution.pitch_rad << ',' << g.x() << ',' << g.y() << ',' << o.fire.fire << ',' << o.fire.status << ','
               << o.bullet_speed_mps << '\n';
    }
  };
  for(const auto& e:events) {
    while(next_tick<e.recv_ns) {tick(next_tick); next_tick+=period;}
    if(auto* m=std::get_if<Imu>(&e.message)) {gimbal=quaternion(*m); engine.push_orientation(stamp(m->header.stamp),gimbal);}
    else if(auto* m=std::get_if<State>(&e.message)) {
      bof::GimbalFeedback f; f.stamp_ns=stamp(m->header.stamp); f.mode=m->mode; f.bullet_speed=m->bullet_speed_mps;
      f.bullet_count=static_cast<int>(m->bullet_count); engine.push_feedback(f,false);
    } else if(auto* m=std::get_if<DirectiveMsg>(&e.message)) {
      bof::Directive d; d.aim_enable=m->aim_enable; d.fire_enable=m->fire_enable;
      for(int l:m->excluded_labels) d.policy.excluded.insert(l);
      d.policy.lock_label=m->lock_label; d.policy.priority_label=m->priority_label; d.policy.outpost_stopped=m->outpost_stopped;
      engine.set_directive(d,e.recv_ns);
    } else if(auto* m=std::get_if<Poses>(&e.message)) log_updates(engine.enqueue(frame_of(*m),e.recv_ns));
  }
  const auto counters=engine.counters();
  std::cout << "replay: " << updates << " tracker updates, " << steps << " command steps at " << rate_hz << " Hz, "
            << fire << " fire steps, sync_drops=" << counters.sync_drops << ", rejected_imu=" << counters.rejected_imu << "\n";
  for(const auto& [id,t]:spin_time) std::cout << "  track " << id << ": spin " << spin_value[id] << " decided at " << t << " s\n";
  std::cout << "  a-priori xy residual (valid tracks): mean " << residual_xy.mean() << " m, sd " << residual_xy.sd() << " m, n=" << residual_xy.n << "\n";
  for(int k=0;k<2;++k) for(const auto& [plate,s]:residual_z[k])
    std::cout << "  " << (k?"outpost":"vehicle") << " plate " << plate << " z residual: mean " << s.mean() << " m, sd " << s.sd() << " m, n=" << s.n << "\n";
  std::cout << "  fire status while solution valid:";
  for(const auto& [k,v]:fire_status) std::cout << ' ' << k << '=' << v;
  std::cout << "\n  wrote " << out << "/tracks.csv and commands.csv\n";
}
// Static target, moving gimbal: the latency that makes the target stand still in the reference frame.
void latency_sweep(const std::vector<Event>& events,const bof::AimConfig& config,double lo,double hi,double step,int label) {
  bof::OrientationBuffer buffer(bof::ClockDomain::ros,static_cast<std::int64_t>(config.tracking.imu_gap_s*1e9),1<<22);
  std::vector<bof::PoseFrame> frames;
  for(const auto& e:events) {
    if(auto* m=std::get_if<Imu>(&e.message)) buffer.push({stamp(m->header.stamp),quaternion(*m)});
    else if(auto* m=std::get_if<Poses>(&e.message)) frames.push_back(frame_of(*m));
  }
  std::cout << "latency sweep over " << frames.size() << " pose frames (target must be static; move the gimbal during recording)\n";
  double best=NAN, best_cost=1e18;
  for(double latency=lo;latency<=hi+1e-12;latency+=step) {
    Stat yaw, pitch; Eigen::Vector2d ref(NAN,NAN);
    for(const auto& f:frames) {
      const auto q=buffer.at(f.stamp_ns-static_cast<std::int64_t>(latency*1e9),bof::ClockDomain::ros);
      if(!q) continue;
      for(const auto& p:f.poses) {
        if(label>=0 && p.label!=label) continue;
        bof::SpatialObservation o;
        if(!bof::to_reference(config,*q,p,o)) continue;
        const double y=std::atan2(o.position.y(),o.position.x()), e=std::atan2(o.position.z(),o.position.head<2>().norm());
        if(!std::isfinite(ref.x())) ref={y,e};
        yaw.add(std::remainder(y-ref.x(),2*pi)); pitch.add(e-ref.y());
      }
    }
    if(yaw.n<20) continue;
    const double cost=yaw.sd()*yaw.sd()+pitch.sd()*pitch.sd();
    std::cout << std::fixed << std::setprecision(4) << "  latency " << latency << " s: bearing sd " << yaw.sd()*180/pi
              << " deg, elevation sd " << pitch.sd()*180/pi << " deg, n=" << yaw.n << "\n";
    if(cost<best_cost) {best_cost=cost; best=latency;}
  }
  if(std::isfinite(best)) std::cout << "best camera_latency_s ~= " << best << " (pose stamp minus exposure; fill aim.camera_latency_s)\n";
  else std::cout << "not enough observations with attitude coverage\n";
}
// Command vs measured gimbal pointing lag, and fire command -> bullet_count increment delay.
void command_lag(const std::vector<Event>& events) {
  std::vector<std::pair<std::int64_t,Eigen::Vector2d>> attitude, command;
  std::vector<std::int64_t> fire_edges, shots; bool last_fire=false; long last_count=-1;
  for(const auto& e:events) {
    if(auto* m=std::get_if<Imu>(&e.message)) attitude.push_back({stamp(m->header.stamp),pointing(quaternion(*m))});
    else if(auto* m=std::get_if<Command>(&e.message)) {
      if(m->control) command.push_back({stamp(m->header.stamp),{m->yaw,m->pitch}});
      if(m->fire && !last_fire) fire_edges.push_back(stamp(m->header.stamp));
      last_fire=m->fire;
    } else if(auto* m=std::get_if<State>(&e.message)) {
      if(last_count>=0 && static_cast<long>(m->bullet_count)>last_count) shots.push_back(stamp(m->header.stamp));
      last_count=m->bullet_count;
    }
  }
  if(attitude.size()<10 || command.size()<10) {std::cout << "command lag: need /gimbal/imu and /gimbal/command with control=true\n";}
  else {
    auto at=[&](std::int64_t t)->std::optional<Eigen::Vector2d> {
      auto it=std::lower_bound(attitude.begin(),attitude.end(),t,[](const auto& a,std::int64_t v){return a.first<v;});
      if(it==attitude.end() || it==attitude.begin()) return std::nullopt;
      auto prev=std::prev(it); const double r=double(t-prev->first)/double(it->first-prev->first);
      Eigen::Vector2d d=it->second-prev->second; d.x()=std::remainder(d.x(),2*pi);
      return prev->second+r*d;
    };
    double best=NAN, best_rms=1e18;
    for(int ms=0;ms<=300;++ms) {
      Stat err; for(const auto& [t,c]:command) {
        auto g=at(t+ms*1000000LL); if(!g) continue;
        err.add(std::pow(std::remainder(g->x()-c.x(),2*pi),2)+std::pow(g->y()-c.y(),2));
      }
      if(err.n>10 && err.mean()<best_rms) {best_rms=err.mean(); best=ms*1e-3;}
    }
    std::cout << "gimbal follows the command with lag ~= " << best << " s (rms pointing error " << std::sqrt(best_rms)*180/pi
              << " deg at that lag). Part of aimer.control_delay_s; the rest is trigger-to-muzzle time.\n";
  }
  std::vector<double> delays;
  for(auto f:fire_edges) {
    auto it=std::lower_bound(shots.begin(),shots.end(),f);
    if(it!=shots.end() && (*it-f)*1e-9<0.5) delays.push_back((*it-f)*1e-9);
  }
  std::cout << "fire edges " << fire_edges.size() << ", matched bullet_count increments " << delays.size()
            << ", median fire->count delay " << median(delays) << " s (meaning depends on when the electrical side counts a shot)\n";
}
}
int main(int argc,char** argv) {
  try {
    std::string bag, config_path, output="aim_replay_output", mode="replay"; double lo=0, hi=0.06, step=0.001, rate=500; int label=-1;
    for(int i=1;i<argc;++i) {
      const std::string a=argv[i]; auto next=[&]{if(i+1>=argc) throw std::invalid_argument("missing value for "+a); return std::string(argv[++i]);};
      if(a=="--bag") bag=next(); else if(a=="--config") config_path=next(); else if(a=="--output") output=next();
      else if(a=="--mode") mode=next(); else if(a=="--rate") rate=std::stod(next()); else if(a=="--label") label=std::stoi(next());
      else if(a=="--min") lo=std::stod(next()); else if(a=="--max") hi=std::stod(next()); else if(a=="--step") step=std::stod(next());
      else if(a=="--help" || a=="-h") {
        std::cout << "bof_aim_replay --bag BAG --config sentry.yaml [--mode replay|latency|command_lag] [--output DIR]\n"
          " replay:      drive AimEngine with recorded poses/attitude; tracks.csv, commands.csv, z residual per plate\n"
          " latency:     static target, gimbal moving; sweep --min/--max/--step seconds [--label L]\n"
          " command_lag: gimbal lag behind /gimbal/command and fire->bullet_count delay\n"
          "Record: ros2 bag record /perception/armor_poses /gimbal/imu /gimbal/state /gimbal/command /sentry/aim_directive\n";
        return 0;
      } else throw std::invalid_argument("unknown argument "+a);
    }
    if(bag.empty()) throw std::invalid_argument("--bag is required");
    auto config=bof::load_config(config_path).aim;
    // Replay needs the measured extrinsics and models but not a live gimbal link or enabled flag.
    config.enabled=mode=="replay"; config.gimbal_link.imu_source="topic";
    bof::validate_aim_config(config);
    if(mode=="latency" && (config.extrinsics.camera_to_gimbal_q_xyzw.size()!=4 || config.extrinsics.camera_in_gimbal_m.size()!=3))
      throw std::invalid_argument("latency mode needs aim.extrinsics (run bof_handeye first)");
    const auto events=read_bag(bag);
    std::cout << "read " << events.size() << " messages from " << bag << "\n";
    if(mode=="replay") {std::filesystem::create_directories(output); replay(events,config,output,rate);}
    else if(mode=="latency") {
      if(!(step>0) || hi<lo) throw std::invalid_argument("invalid sweep range");
      config.timestamp_source="exposure_synchronized"; config.camera_latency_s=0; latency_sweep(events,config,lo,hi,step,label);
    } else if(mode=="command_lag") command_lag(events);
    else throw std::invalid_argument("mode must be replay, latency or command_lag");
  } catch(const std::exception& e) {std::cerr << "bof_aim_replay: " << e.what() << "\n"; return 1;}
  return 0;
}
