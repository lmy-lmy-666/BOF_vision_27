#pragma once
#include "bof_vision/detector_factory.hpp"
#include "bof_vision/pose.hpp"
#include "bof_vision/aim.hpp"
#include <string>
#include <map>

namespace bof {
struct YoloProfile {
  std::string backend, model_path, device="CPU";
  float confidence=0.7F, nms=0.3F;
  std::string classifier;              // tongji_yolov8 number classifier (tiny_resnet.onnx)
  float classifier_confidence=0.8F;
};
struct LiveConfig {
  std::string source, video_path, camera_serial="auto", output;
  int usb_index=0, frames=0, retry_delay_ms=1000, model_cache_size=2;
  double fps=0, exposure_us=3000, gain=0, record_fps=30, reconnect_timeout_s=30;
  bool headless=false, record=false, reconnect=true;
};
struct PerceptionConfig {
  DetectorConfig detector;
  std::string yolo_name;
  std::map<std::string,YoloProfile> yolo_models;
  LiveConfig live;
  AimConfig aim;
  double track_timeout_s = 0.2;
  double association_gate_px = 100.0;
  AssociationConfig association;
  bool publish_debug_image = true;
  PoseConfig pose;
};
// Load defaults, then an optional base file, then this file. Validate after overrides.
// YAML model paths are relative to the file declaring them; CLI/ROS overrides use cwd.
PerceptionConfig load_config(const std::string& path = "");
void select_yolo(PerceptionConfig& config, const std::string& name);
void validate_config(PerceptionConfig& config);
std::string config_yaml(const PerceptionConfig& config);
void save_config(const PerceptionConfig& config, const std::string& path);
Color parse_color(const std::string& value);
const char* color_name(Color color);
}  // namespace bof
