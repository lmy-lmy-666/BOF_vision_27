// CRC16 table and frame handling adapted from Tongji sp_vision_25 tools/crc.cpp and io/gimbal/gimbal.cpp (MIT);
// cboard layout and angle conventions from the team firmware Task/Src/Auto_aim.c and GimbolControl.c.
#include "bof_vision/gimbal_link.hpp"
#include "bof_vision/aim.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <map>
#include <stdexcept>

namespace bof {
namespace {
const std::uint16_t table[256]={
  0x0000,0x1189,0x2312,0x329b,0x4624,0x57ad,0x6536,0x74bf,0x8c48,0x9dc1,0xaf5a,0xbed3,0xca6c,0xdbe5,0xe97e,0xf8f7,
  0x1081,0x0108,0x3393,0x221a,0x56a5,0x472c,0x75b7,0x643e,0x9cc9,0x8d40,0xbfdb,0xae52,0xdaed,0xcb64,0xf9ff,0xe876,
  0x2102,0x308b,0x0210,0x1399,0x6726,0x76af,0x4434,0x55bd,0xad4a,0xbcc3,0x8e58,0x9fd1,0xeb6e,0xfae7,0xc87c,0xd9f5,
  0x3183,0x200a,0x1291,0x0318,0x77a7,0x662e,0x54b5,0x453c,0xbdcb,0xac42,0x9ed9,0x8f50,0xfbef,0xea66,0xd8fd,0xc974,
  0x4204,0x538d,0x6116,0x709f,0x0420,0x15a9,0x2732,0x36bb,0xce4c,0xdfc5,0xed5e,0xfcd7,0x8868,0x99e1,0xab7a,0xbaf3,
  0x5285,0x430c,0x7197,0x601e,0x14a1,0x0528,0x37b3,0x263a,0xdecd,0xcf44,0xfddf,0xec56,0x98e9,0x8960,0xbbfb,0xaa72,
  0x6306,0x728f,0x4014,0x519d,0x2522,0x34ab,0x0630,0x17b9,0xef4e,0xfec7,0xcc5c,0xddd5,0xa96a,0xb8e3,0x8a78,0x9bf1,
  0x7387,0x620e,0x5095,0x411c,0x35a3,0x242a,0x16b1,0x0738,0xffcf,0xee46,0xdcdd,0xcd54,0xb9eb,0xa862,0x9af9,0x8b70,
  0x8408,0x9581,0xa71a,0xb693,0xc22c,0xd3a5,0xe13e,0xf0b7,0x0840,0x19c9,0x2b52,0x3adb,0x4e64,0x5fed,0x6d76,0x7cff,
  0x9489,0x8500,0xb79b,0xa612,0xd2ad,0xc324,0xf1bf,0xe036,0x18c1,0x0948,0x3bd3,0x2a5a,0x5ee5,0x4f6c,0x7df7,0x6c7e,
  0xa50a,0xb483,0x8618,0x9791,0xe32e,0xf2a7,0xc03c,0xd1b5,0x2942,0x38cb,0x0a50,0x1bd9,0x6f66,0x7eef,0x4c74,0x5dfd,
  0xb58b,0xa402,0x9699,0x8710,0xf3af,0xe226,0xd0bd,0xc134,0x39c3,0x284a,0x1ad1,0x0b58,0x7fe7,0x6e6e,0x5cf5,0x4d7c,
  0xc60c,0xd785,0xe51e,0xf497,0x8028,0x91a1,0xa33a,0xb2b3,0x4a44,0x5bcd,0x6956,0x78df,0x0c60,0x1de9,0x2f72,0x3efb,
  0xd68d,0xc704,0xf59f,0xe416,0x90a9,0x8120,0xb3bb,0xa232,0x5ac5,0x4b4c,0x79d7,0x685e,0x1ce1,0x0d68,0x3ff3,0x2e7a,
  0xe70e,0xf687,0xc41c,0xd595,0xa12a,0xb0a3,0x8238,0x93b1,0x6b46,0x7acf,0x4854,0x59dd,0x2d62,0x3ceb,0x0e70,0x1ff9,
  0xf78f,0xe606,0xd49d,0xc514,0xb1ab,0xa022,0x92b9,0x8330,0x7bc7,0x6a4e,0x58d5,0x495c,0x3de3,0x2c6a,0x1ef1,0x0f78};
speed_t baud_constant(int baud) {
  static const std::map<int,speed_t> rates{{9600,B9600},{57600,B57600},{115200,B115200},{230400,B230400},
    {460800,B460800},{921600,B921600},{1000000,B1000000},{2000000,B2000000}};
  auto it=rates.find(baud);
  if(it==rates.end()) throw std::invalid_argument("unsupported serial baud rate");
  return it->second;
}
}
std::uint16_t crc16(const std::uint8_t* data,std::size_t size) {
  std::uint16_t crc=0xffff;
  while(size--) crc=(crc>>8)^table[(crc^*data++)&0xff];
  return crc;
}
std::vector<std::uint8_t> encode(sp::VisionToGimbal frame) {
  frame.head[0]='S'; frame.head[1]='P';
  frame.crc16=crc16(reinterpret_cast<const std::uint8_t*>(&frame),sizeof(frame)-sizeof(frame.crc16));
  const auto* bytes=reinterpret_cast<const std::uint8_t*>(&frame);
  return {bytes,bytes+sizeof(frame)};
}
std::optional<sp::GimbalToVision> decode(const std::uint8_t* data,std::size_t size) {
  if(size!=sizeof(sp::GimbalToVision) || data[0]!='S' || data[1]!='P') return std::nullopt;
  const std::uint16_t expected=static_cast<std::uint16_t>(data[size-2]|(data[size-1]<<8));
  if(crc16(data,size-2)!=expected) return std::nullopt;
  sp::GimbalToVision frame;
  std::memcpy(&frame,data,size);
  return frame;
}
namespace {
template<class T> std::vector<std::uint8_t> bytes_with_crc(T frame) {
  frame.crc16=crc16(reinterpret_cast<const std::uint8_t*>(&frame),sizeof(frame)-sizeof(frame.crc16));
  const auto* bytes=reinterpret_cast<const std::uint8_t*>(&frame);
  return {bytes,bytes+sizeof(frame)};
}
template<class T> std::optional<T> checked(const std::uint8_t* data) {
  constexpr std::size_t size=sizeof(T);
  if(crc16(data,size-2)!=static_cast<std::uint16_t>(data[size-2]|(data[size-1]<<8))) return std::nullopt;
  T frame; std::memcpy(&frame,data,size); return frame;
}
}
LinkProtocol sp_protocol(bool up) {
  const double sign=up?1:-1;
  LinkProtocol p{"sp",{'S','P'},sizeof(sp::GimbalToVision),{},{}};
  p.decode=[sign](const std::uint8_t* data)->std::optional<GimbalFeedback> {
    auto frame=decode(data,sizeof(sp::GimbalToVision));
    if(!frame) return std::nullopt;
    GimbalFeedback f; f.orientation=Eigen::Quaterniond(frame->q[0],frame->q[1],frame->q[2],frame->q[3]);
    f.mode=frame->mode; f.yaw=frame->yaw; f.yaw_vel=frame->yaw_vel; f.pitch=sign*frame->pitch; f.pitch_vel=sign*frame->pitch_vel;
    f.bullet_speed=frame->bullet_speed; f.bullet_count=frame->bullet_count;
    return f;
  };
  p.encode=[sign](const GimbalCommand& c) {
    sp::VisionToGimbal frame{};
    frame.mode=c.control?(c.fire?2:1):0;
    frame.yaw=static_cast<float>(c.yaw); frame.yaw_vel=static_cast<float>(c.yaw_vel); frame.yaw_acc=static_cast<float>(c.yaw_acc);
    frame.pitch=static_cast<float>(sign*c.pitch); frame.pitch_vel=static_cast<float>(sign*c.pitch_vel);
    frame.pitch_acc=static_cast<float>(sign*c.pitch_acc);
    return encode(frame);
  };
  return p;
}
LinkProtocol cboard_protocol() {
  LinkProtocol p{"cboard",{0x53},sizeof(cboard::GimbalToVision),{},{}};
  p.decode=[](const std::uint8_t* data)->std::optional<GimbalFeedback> {
    auto frame=checked<cboard::GimbalToVision>(data);
    if(!frame) return std::nullopt;
    GimbalFeedback f; f.orientation=Eigen::Quaterniond(frame->q[0],frame->q[1],frame->q[2],frame->q[3]);
    f.mode=frame->mode; f.enemy_color=frame->enemy_color?1:0;
    // Firmware yaw = INS.Yaw (heading of IMU x); pitch = INS.Roll, the rotation about IMU y, nose down positive.
    f.yaw=frame->yaw; f.pitch=-frame->pitch;
    f.yaw_vel=frame->yaw_vel; f.pitch_vel=frame->pitch_vel;   // motor speeds, not rad/s
    f.bullet_speed=frame->bullet_speed; f.bullet_count=frame->bullet_count;
    return f;
  };
  p.encode=[](const GimbalCommand& c) {
    // Gimbol_AUTO: INS.Yaw -> yaw*57.3 deg, INS.Roll -> -pitch*57.3 deg, so pitch is elevation, up positive.
    cboard::VisionToGimbal frame{};
    frame.mode=c.control?(c.fire?2:1):0;
    frame.yaw=static_cast<float>(c.yaw); frame.yaw_vel=static_cast<float>(c.yaw_vel); frame.yaw_acc=static_cast<float>(c.yaw_acc);
    frame.pitch=static_cast<float>(c.pitch); frame.pitch_vel=static_cast<float>(c.pitch_vel); frame.pitch_acc=static_cast<float>(c.pitch_acc);
    return bytes_with_crc(frame);
  };
  return p;
}
SerialGimbalLink::SerialGimbalLink(std::string port,int baud,LinkProtocol protocol,Clock clock,Callback callback)
: port_(std::move(port)),baud_(baud),protocol_(std::move(protocol)),clock_(std::move(clock)),callback_(std::move(callback)) {
  baud_constant(baud_);
  open_port();
  thread_=std::thread([this]{read_loop();});
}
SerialGimbalLink::~SerialGimbalLink() {
  quit_=true;
  if(thread_.joinable()) thread_.join();
  close_port();
}
void SerialGimbalLink::open_port() {
  const int fd=::open(port_.c_str(),O_RDWR|O_NOCTTY|O_CLOEXEC);
  if(fd<0) return;
  termios tty{};
  if(tcgetattr(fd,&tty)!=0) {::close(fd); return;}
  cfmakeraw(&tty);
  cfsetispeed(&tty,baud_constant(baud_)); cfsetospeed(&tty,baud_constant(baud_));   // ignored by USB CDC
  tty.c_cflag|=CLOCAL|CREAD; tty.c_cflag&=~CRTSCTS;
  tty.c_cc[VMIN]=0; tty.c_cc[VTIME]=1;   // 100 ms read timeout lets the thread notice quit_
  if(tcsetattr(fd,TCSANOW,&tty)!=0) {::close(fd); return;}
  tcflush(fd,TCIOFLUSH);
  std::lock_guard<std::mutex> lock(write_mutex_);
  fd_=fd;
}
void SerialGimbalLink::close_port() {
  std::lock_guard<std::mutex> lock(write_mutex_);
  if(fd_>=0) ::close(fd_);
  fd_=-1;
}
bool SerialGimbalLink::send(const GimbalCommand& c) {
  const auto bytes=protocol_.encode(c);
  std::lock_guard<std::mutex> lock(write_mutex_);
  if(fd_<0) return false;
  return ::write(fd_,bytes.data(),bytes.size())==static_cast<ssize_t>(bytes.size());
}
void SerialGimbalLink::read_loop() {
  std::vector<std::uint8_t> buffer;
  std::uint8_t chunk[256];
  auto last_frame=std::chrono::steady_clock::now();
  const auto& head=protocol_.head; const std::size_t size=protocol_.feedback_size;
  while(!quit_) {
    if(fd_<0 || std::chrono::steady_clock::now()-last_frame>std::chrono::seconds(2)) {
      // No valid frame for 2 s or port lost: reopen (Tongji Gimbal::reconnect).
      close_port(); std::this_thread::sleep_for(std::chrono::milliseconds(500));
      open_port(); buffer.clear(); last_frame=std::chrono::steady_clock::now(); continue;
    }
    const ssize_t n=::read(fd_,chunk,sizeof(chunk));
    if(n<0) {close_port(); continue;}
    buffer.insert(buffer.end(),chunk,chunk+n);
    std::size_t start=0;
    while(buffer.size()-start>=size) {
      if(!std::equal(head.begin(),head.end(),buffer.begin()+static_cast<std::ptrdiff_t>(start))) {++start; continue;}
      auto f=protocol_.decode(buffer.data()+start);
      if(!f) {++crc_errors_; ++start; continue;}
      start+=size; ++frames_; last_frame=std::chrono::steady_clock::now();
      f->stamp_ns=clock_();
      if(callback_) callback_(*f);
    }
    buffer.erase(buffer.begin(),buffer.begin()+static_cast<std::ptrdiff_t>(start));
    if(buffer.size()>4096) buffer.clear();
  }
}
std::unique_ptr<GimbalIO> make_gimbal_link(const GimbalLinkConfig& c,GimbalIO::Clock clock,GimbalIO::Callback callback) {
  if(c.imu_source!="serial") return nullptr;
  auto protocol=c.protocol=="sp"?sp_protocol(c.pitch_up_positive):cboard_protocol();
  return std::make_unique<SerialGimbalLink>(c.serial_port,c.serial_baud,std::move(protocol),std::move(clock),std::move(callback));
}
}
