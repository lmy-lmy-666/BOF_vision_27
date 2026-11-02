#pragma once
// Serial gimbal link. Frame layouts: Tongji sp_vision_25 io/gimbal/gimbal.hpp (MIT) and the team
// gimbal firmware sentry_gimbol_new Task/Inc/Auto_aim.h (one-byte headers, enemy colour byte).
#include <Eigen/Geometry>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace bof {
namespace sp {
struct __attribute__((packed)) GimbalToVision {
  std::uint8_t head[2]={'S','P'};
  std::uint8_t mode;          // 0 idle, 1 auto aim, 2 small buff, 3 big buff
  float q[4];                 // wxyz, gimbal -> reference
  float yaw, yaw_vel, pitch, pitch_vel, bullet_speed;
  std::uint16_t bullet_count;
  std::uint16_t crc16;
};
struct __attribute__((packed)) VisionToGimbal {
  std::uint8_t head[2]={'S','P'};
  std::uint8_t mode;          // 0 no control, 1 control, 2 control and fire
  float yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc;
  std::uint16_t crc16;
};
}
// Team firmware (USB CDC). The gimbal drives INS.Yaw to yaw and its INS.Roll (standard pitch about
// body y, positive nose down) to -pitch, so the command pitch is elevation (up positive) while the
// reported pitch is INS.Roll (up negative).
namespace cboard {
struct __attribute__((packed)) GimbalToVision {
  std::uint8_t head=0x53;
  std::uint8_t mode;          // firmware sends 1
  std::uint8_t enemy_color;   // robot_id<100 (we are red) -> 1, else 0
  float q[4];                 // wxyz, IMU body -> earth (z up)
  float yaw, yaw_vel, pitch, pitch_vel, bullet_speed;   // yaw/pitch rad; *_vel are GM6020 speeds
  std::uint16_t bullet_count;
  std::uint16_t crc16;
};
struct __attribute__((packed)) VisionToGimbal {
  std::uint8_t head=0x50;
  std::uint8_t mode;          // 0 no control, 1 control, 2 control and fire, 3 ignored
  float yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc;
  std::uint16_t crc16;
};
}
std::uint16_t crc16(const std::uint8_t* data, std::size_t size);
// Frame helpers; the CRC covers everything before the trailing crc16 field (little endian).
std::vector<std::uint8_t> encode(sp::VisionToGimbal frame);
std::optional<sp::GimbalToVision> decode(const std::uint8_t* data, std::size_t size);

struct GimbalFeedback {
  std::int64_t stamp_ns=0;    // receive time from the supplied clock
  Eigen::Quaterniond orientation=Eigen::Quaterniond::Identity();
  int mode=0;
  int enemy_color=-1;         // -1 not reported, 0 red, 1 blue (bof::Color order)
  double yaw=0, yaw_vel=0, pitch=0, pitch_vel=0, bullet_speed=0;   // pitch up positive
  int bullet_count=0;
};
struct GimbalCommand {
  bool control=false, fire=false;
  double yaw=0, yaw_vel=0, yaw_acc=0, pitch=0, pitch_vel=0, pitch_acc=0;   // pitch up positive
};
// One wire protocol: frame start, upstream frame size, and the conversions to/from the
// up-positive conventions above.
struct LinkProtocol {
  std::string name;
  std::vector<std::uint8_t> head;
  std::size_t feedback_size=0;
  std::function<std::optional<GimbalFeedback>(const std::uint8_t*)> decode;   // CRC checked, no stamp
  std::function<std::vector<std::uint8_t>(const GimbalCommand&)> encode;
};
LinkProtocol sp_protocol(bool pitch_up_positive);
LinkProtocol cboard_protocol();
class GimbalIO {
public:
  using Clock=std::function<std::int64_t()>;
  using Callback=std::function<void(const GimbalFeedback&)>;
  virtual ~GimbalIO()=default;
  virtual bool send(const GimbalCommand& command)=0;
  virtual bool connected() const=0;
  virtual std::uint64_t frames() const=0;
  virtual std::uint64_t crc_errors() const=0;
};
struct GimbalLinkConfig;
// Returns null when the configuration uses no gimbal link (imu_source=topic).
std::unique_ptr<GimbalIO> make_gimbal_link(const GimbalLinkConfig& config, GimbalIO::Clock clock, GimbalIO::Callback callback);
// POSIX serial link: read thread finds frames, reconnects after errors; send is thread-safe.
class SerialGimbalLink final:public GimbalIO {
public:
  SerialGimbalLink(std::string port, int baud, LinkProtocol protocol, Clock clock, Callback callback);
  ~SerialGimbalLink() override;
  bool send(const GimbalCommand& command) override;
  bool connected() const override {return fd_>=0;}
  std::uint64_t crc_errors() const override {return crc_errors_;}
  std::uint64_t frames() const override {return frames_;}
private:
  void open_port();
  void close_port();
  void read_loop();
  std::string port_;
  int baud_;
  LinkProtocol protocol_;
  Clock clock_;
  Callback callback_;
  std::atomic<int> fd_{-1};
  std::atomic<bool> quit_{false};
  std::atomic<std::uint64_t> crc_errors_{0}, frames_{0};
  std::mutex write_mutex_;
  std::thread thread_;
};
}
