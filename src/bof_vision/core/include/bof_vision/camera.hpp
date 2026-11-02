#pragma once
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <opencv2/core.hpp>

namespace bof {
struct CameraFrame {
  cv::Mat bgr;
  std::uint64_t sequence;
  std::chrono::steady_clock::time_point received;
};
class CameraSource {
public:
  virtual ~CameraSource() = default;
  virtual std::optional<CameraFrame> read(unsigned timeout_ms=200) = 0;
  // Call on the acquisition thread, between reads. Return SDK-clamped applied values.
  virtual double set_exposure_us(double value) = 0;
  virtual double set_gain(double value) = 0;
  virtual std::string serial_number() const { return {}; }
};
bool hik_camera_enabled();
std::unique_ptr<CameraSource> make_hik_camera(const std::string& serial,double exposure_us,double gain=-1);
}  // namespace bof
