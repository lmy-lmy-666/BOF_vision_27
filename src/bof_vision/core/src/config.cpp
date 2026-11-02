#include "bof_vision/config.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <stdexcept>
#include <utility>

namespace bof {
namespace {
namespace fs = std::filesystem;
void keys(const YAML::Node& node, const std::set<std::string>& allowed) {
  if (!node.IsMap()) throw std::invalid_argument("expected a YAML mapping");
  std::set<std::string> seen;
  for (const auto& entry : node) {
    if (!entry.first.IsScalar()) throw std::invalid_argument("configuration keys must be strings");
    const auto key = entry.first.as<std::string>();
    if (!allowed.count(key)) throw std::invalid_argument("unknown configuration field: " + key);
    if (!seen.insert(key).second) throw std::invalid_argument("duplicate configuration field: " + key);
  }
}
template<class T> void field(const YAML::Node& node, const char* key, T& value) {
  if (!node[key]) return;
  if (!node[key].IsScalar()) throw std::invalid_argument(std::string(key) + " must be a scalar");
  try { value = node[key].as<T>(); }
  catch (const YAML::Exception& e) { throw std::invalid_argument(std::string(key) + ": " + e.what()); }
}
std::string absolute_path(const std::string& value, const fs::path& base) {
  return fs::absolute(base / fs::path(value)).lexically_normal().string();
}
void load(const fs::path& path, PerceptionConfig& config, std::set<fs::path>& active) {
  const auto canonical = fs::canonical(path);
  if (active.size() >= 16) throw std::invalid_argument("configuration inheritance exceeds 16 files");
  if (!active.insert(canonical).second) throw std::invalid_argument("configuration inheritance cycle: " + path.string());
  try {
    auto root = YAML::LoadFile(path.string());
    // Also accept the original ROS parameter file for migration.
    if (root.IsMap() && root["pipeline"]) {
      keys(root, {"pipeline"});
      keys(root["pipeline"], {"ros__parameters"});
      root = YAML::Clone(root["pipeline"]["ros__parameters"]);
    }
    keys(root, {"schema_version", "extends", "enemy_color", "detector_backend", "model_path",
      "inference_device", "color_threshold", "confidence_threshold", "nms_threshold",
      "track_timeout_s", "association_gate_px", "publish_debug_image", "calibration_path",
      "plate_width_m", "plate_height_m", "pose_label", "max_reprojection_error_px", "geometry_path",
      "track_confirm_frames", "track_label_confirm_frames", "association_ambiguity_px",
      "yolo_name", "yolo_models", "live", "aim"});
    int version = 1;
    field(root, "schema_version", version);
    if (version != 1) throw std::invalid_argument("unsupported schema_version (expected 1)");
    std::string parent;
    field(root, "extends", parent);
    if (root["extends"] && parent.empty()) throw std::invalid_argument("extends must name a file");
    if (!parent.empty()) load(canonical.parent_path() / parent, config, active);
    if(root["aim"]) load_aim_config(root["aim"],config.aim);
    if(root["yolo_models"]) {
      const auto models=root["yolo_models"];
      if(!models.IsMap()) throw std::invalid_argument("yolo_models must be a mapping");
      std::set<std::string> names;
      for(const auto& entry:models) {
        const auto name=entry.first.as<std::string>();
        if(name.empty() || name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos || !names.insert(name).second)
          throw std::invalid_argument("invalid/duplicate yolo profile name: "+name);
        const auto node=entry.second;
        keys(node,{"backend","model_path","device","confidence","nms","classifier","classifier_confidence"});
        YoloProfile profile;
        field(node,"backend",profile.backend); field(node,"model_path",profile.model_path);
        field(node,"device",profile.device); field(node,"confidence",profile.confidence); field(node,"nms",profile.nms);
        field(node,"classifier",profile.classifier); field(node,"classifier_confidence",profile.classifier_confidence);
        if(!profile.classifier.empty()) profile.classifier=absolute_path(profile.classifier,canonical.parent_path());
        if(profile.backend.empty()) throw std::invalid_argument("yolo profile requires backend: "+name);
        if(!profile.model_path.empty()) profile.model_path=absolute_path(profile.model_path,canonical.parent_path());
        config.yolo_models[name]=profile;  // An inherited profile is replaced as one complete bundle.
      }
    }
    field(root,"yolo_name",config.yolo_name);
    if(!config.yolo_name.empty() && (root["yolo_name"] ||
       (root["yolo_models"] && root["yolo_models"][config.yolo_name]))) select_yolo(config,config.yolo_name);
    if(root["live"]) {
      const auto node=root["live"]; auto& live=config.live;
      keys(node,{"source","video_path","camera_serial","usb_index","frames","fps","exposure_us","gain",
        "headless","record","record_fps","reconnect","reconnect_timeout_s","retry_delay_ms","output","model_cache_size"});
      field(node,"source",live.source); field(node,"video_path",live.video_path);
      if(node["video_path"] && !live.video_path.empty()) live.video_path=absolute_path(live.video_path,canonical.parent_path());
      field(node,"camera_serial",live.camera_serial); field(node,"usb_index",live.usb_index);
      field(node,"frames",live.frames); field(node,"fps",live.fps); field(node,"exposure_us",live.exposure_us);
      field(node,"gain",live.gain); field(node,"headless",live.headless); field(node,"record",live.record);
      field(node,"record_fps",live.record_fps); field(node,"reconnect",live.reconnect);
      field(node,"reconnect_timeout_s",live.reconnect_timeout_s); field(node,"retry_delay_ms",live.retry_delay_ms);
      field(node,"model_cache_size",live.model_cache_size); field(node,"output",live.output);
      if(node["output"] && !live.output.empty()) live.output=absolute_path(live.output,canonical.parent_path());
    }
    auto& d = config.detector;
    std::string color = color_name(d.enemy_color);
    field(root, "enemy_color", color);
    d.enemy_color = parse_color(color);
    field(root, "detector_backend", d.backend);
    field(root, "model_path", d.model_path);
    if (root["model_path"] && !d.model_path.empty())
      d.model_path = absolute_path(d.model_path, canonical.parent_path());
    field(root, "inference_device", d.device);
    field(root, "color_threshold", d.color_threshold);
    field(root, "confidence_threshold", d.confidence_threshold);
    field(root, "nms_threshold", d.nms_threshold);
    field(root, "track_timeout_s", config.track_timeout_s);
    field(root, "association_gate_px", config.association_gate_px);
    field(root, "track_confirm_frames", config.association.confirm_frames);
    field(root, "track_label_confirm_frames", config.association.label_confirm_frames);
    field(root, "association_ambiguity_px", config.association.ambiguity_px);
    field(root, "publish_debug_image", config.publish_debug_image);
    field(root, "calibration_path", config.pose.calibration_path);
    if(root["calibration_path"] && !config.pose.calibration_path.empty())
      config.pose.calibration_path=absolute_path(config.pose.calibration_path,canonical.parent_path());
    field(root, "plate_width_m", config.pose.plate_width_m);
    field(root, "plate_height_m", config.pose.plate_height_m);
    field(root, "pose_label", config.pose.label);
    field(root, "max_reprojection_error_px", config.pose.max_reprojection_error_px);
    field(root, "geometry_path", config.pose.geometry_path);
    if(root["geometry_path"] && !config.pose.geometry_path.empty())
      config.pose.geometry_path=absolute_path(config.pose.geometry_path,canonical.parent_path());
  } catch (const std::exception& e) {
    throw std::invalid_argument(path.string() + ": " + e.what());
  }
  active.erase(canonical);
}
void range(double value, double low, double high, const char* name) {
  if (!std::isfinite(value) || value < low || value > high)
    throw std::invalid_argument(std::string(name) + " must be finite and in [" +
      std::to_string(low) + ", " + std::to_string(high) + "]");
}
}
Color parse_color(const std::string& value) {
  if (value == "red") return Color::red;
  if (value == "blue") return Color::blue;
  throw std::invalid_argument("enemy_color must be red or blue");
}
const char* color_name(Color color) { return color == Color::red ? "red" : "blue"; }
void select_yolo(PerceptionConfig& config,const std::string& name) {
  const auto it=config.yolo_models.find(name);
  if(it==config.yolo_models.end()) throw std::invalid_argument("unknown yolo_name: "+name);
  const auto& profile=it->second;
  auto& d=config.detector;
  d.backend=profile.backend; d.model_path=profile.model_path; d.device=profile.device;
  d.confidence_threshold=profile.confidence; d.nms_threshold=profile.nms;
  d.classifier_path=profile.classifier; d.classifier_confidence=profile.classifier_confidence;
  config.yolo_name=name;
}
PerceptionConfig load_config(const std::string& path) {
  PerceptionConfig config;
  std::set<fs::path> active;
  if (!path.empty()) load(fs::absolute(path).lexically_normal(), config, active);
  return config;
}
void validate_config(PerceptionConfig& config) {
  validate_aim_config(config.aim);
  const auto& live=config.live;
  if(!live.source.empty() && live.source!="hik" && live.source!="usb" && live.source!="video" && live.source!="synthetic")
    throw std::invalid_argument("live.source must be hik/usb/video/synthetic");
  range(live.usb_index,0,100000,"live.usb_index"); range(live.frames,0,2147483647,"live.frames");
  range(live.fps,0,1000,"live.fps");
  if(live.fps>0 && live.fps<1) throw std::invalid_argument("live.fps must be 0 or >=1");
  if(!std::isfinite(live.exposure_us) || live.exposure_us<=0 || !std::isfinite(live.record_fps) || live.record_fps<=0 || live.record_fps>1000)
    throw std::invalid_argument("live exposure/record FPS must be positive and record FPS <=1000");
  if(!std::isfinite(live.gain) || live.gain<0 || !std::isfinite(live.reconnect_timeout_s) || live.reconnect_timeout_s<0)
    throw std::invalid_argument("live gain/reconnect timeout must be finite and nonnegative");
  range(live.retry_delay_ms,50,60000,"live.retry_delay_ms"); range(live.model_cache_size,0,4,"live.model_cache_size");
  auto& d = config.detector;
  if (d.backend != "light_pair" && d.backend != "tongji_yolov5" && d.backend != "tongji_yolov8" &&
      d.backend != "tongji_yolo11" && d.backend != "shenzhen_0526" && d.backend != "shenzhen_0708")
    throw std::invalid_argument("unknown detector_backend: " + d.backend);
  if (d.device.empty()) throw std::invalid_argument("inference_device must not be empty");
  range(d.color_threshold, 0, 255, "color_threshold");
  for (const auto& threshold : {std::pair{"confidence_threshold", d.confidence_threshold},
                                std::pair{"nms_threshold", d.nms_threshold}})
    if (!std::isfinite(threshold.second) || threshold.second <= 0 || threshold.second >= 1)
      throw std::invalid_argument(std::string(threshold.first) + " must be finite and in (0,1)");
  if (!std::isfinite(config.track_timeout_s) || config.track_timeout_s <= 0)
    throw std::invalid_argument("track_timeout_s must be finite and positive");
  if (!std::isfinite(config.association_gate_px) || config.association_gate_px <= 0)
    throw std::invalid_argument("association_gate_px must be finite and positive");
  range(config.association.confirm_frames, 1, 1000, "track_confirm_frames");
  range(config.association.label_confirm_frames, 1, 1000, "track_label_confirm_frames");
  if(!std::isfinite(config.association.ambiguity_px) || config.association.ambiguity_px<0 ||
     config.association.ambiguity_px>=config.association_gate_px)
    throw std::invalid_argument("association_ambiguity_px must be in [0, association_gate_px)");
  if (!d.model_path.empty()) d.model_path = absolute_path(d.model_path, fs::current_path());
  if (d.backend != "light_pair" && (d.model_path.empty() || !fs::is_regular_file(d.model_path)))
    throw std::invalid_argument("model_path must name an existing model for " + d.backend);
  if (!d.classifier_path.empty()) d.classifier_path = absolute_path(d.classifier_path, fs::current_path());
  if (d.backend == "tongji_yolov8" && (d.classifier_path.empty() || !fs::is_regular_file(d.classifier_path)))
    throw std::invalid_argument("tongji_yolov8 needs an existing classifier (tiny_resnet.onnx)");
  if (!std::isfinite(d.classifier_confidence) || d.classifier_confidence<=0 || d.classifier_confidence>=1)
    throw std::invalid_argument("classifier_confidence must be in (0,1)");
  if(!config.pose.calibration_path.empty()) {
    config.pose.calibration_path=absolute_path(config.pose.calibration_path,fs::current_path());
    if(!config.pose.geometry_path.empty()) config.pose.geometry_path=absolute_path(config.pose.geometry_path,fs::current_path());
    // Validate calibration and explicit geometry without opening a camera or inference device.
    PoseSolver checked(config.pose);
  } else if(!config.pose.geometry_path.empty() || config.pose.plate_width_m!=0 || config.pose.plate_height_m!=0 || config.pose.label!=-2) {
    throw std::invalid_argument("pose geometry was supplied without calibration_path");
  }
}
std::string config_yaml(const PerceptionConfig& config) {
  YAML::Emitter out;
  out.SetFloatPrecision(9);
  out.SetDoublePrecision(17);
  const auto& d = config.detector;
  out << YAML::BeginMap
      << YAML::Key << "schema_version" << YAML::Value << 1
      << YAML::Key << "yolo_name" << YAML::Value << config.yolo_name
      << YAML::Key << "yolo_models" << YAML::Value << YAML::BeginMap;
  for(const auto& [name,p]:config.yolo_models)
    out << YAML::Key << name << YAML::Value << YAML::BeginMap
        << YAML::Key << "backend" << YAML::Value << p.backend << YAML::Key << "model_path" << YAML::Value << p.model_path
        << YAML::Key << "device" << YAML::Value << p.device << YAML::Key << "confidence" << YAML::Value << p.confidence
        << YAML::Key << "nms" << YAML::Value << p.nms << YAML::Key << "classifier" << YAML::Value << p.classifier
        << YAML::Key << "classifier_confidence" << YAML::Value << p.classifier_confidence << YAML::EndMap;
  const auto& live=config.live;
  out << YAML::EndMap << YAML::Key << "live" << YAML::Value << YAML::BeginMap
      << YAML::Key << "source" << YAML::Value << live.source << YAML::Key << "video_path" << YAML::Value << live.video_path
      << YAML::Key << "camera_serial" << YAML::Value << live.camera_serial << YAML::Key << "usb_index" << YAML::Value << live.usb_index
      << YAML::Key << "frames" << YAML::Value << live.frames << YAML::Key << "fps" << YAML::Value << live.fps
      << YAML::Key << "exposure_us" << YAML::Value << live.exposure_us << YAML::Key << "gain" << YAML::Value << live.gain
      << YAML::Key << "headless" << YAML::Value << live.headless << YAML::Key << "record" << YAML::Value << live.record
      << YAML::Key << "record_fps" << YAML::Value << live.record_fps << YAML::Key << "reconnect" << YAML::Value << live.reconnect
      << YAML::Key << "reconnect_timeout_s" << YAML::Value << live.reconnect_timeout_s
      << YAML::Key << "retry_delay_ms" << YAML::Value << live.retry_delay_ms << YAML::Key << "output" << YAML::Value << live.output
      << YAML::Key << "model_cache_size" << YAML::Value << live.model_cache_size << YAML::EndMap
      << YAML::Key << "enemy_color" << YAML::Value << color_name(d.enemy_color)
      << YAML::Key << "detector_backend" << YAML::Value << d.backend
      << YAML::Key << "model_path" << YAML::Value << d.model_path
      << YAML::Key << "inference_device" << YAML::Value << d.device
      << YAML::Key << "color_threshold" << YAML::Value << d.color_threshold
      << YAML::Key << "confidence_threshold" << YAML::Value << d.confidence_threshold
      << YAML::Key << "nms_threshold" << YAML::Value << d.nms_threshold
      << YAML::Key << "track_timeout_s" << YAML::Value << config.track_timeout_s
      << YAML::Key << "association_gate_px" << YAML::Value << config.association_gate_px
      << YAML::Key << "track_confirm_frames" << YAML::Value << config.association.confirm_frames
      << YAML::Key << "track_label_confirm_frames" << YAML::Value << config.association.label_confirm_frames
      << YAML::Key << "association_ambiguity_px" << YAML::Value << config.association.ambiguity_px
      << YAML::Key << "publish_debug_image" << YAML::Value << config.publish_debug_image
      << YAML::Key << "calibration_path" << YAML::Value << config.pose.calibration_path
      << YAML::Key << "geometry_path" << YAML::Value << config.pose.geometry_path
      << YAML::Key << "plate_width_m" << YAML::Value << config.pose.plate_width_m
      << YAML::Key << "plate_height_m" << YAML::Value << config.pose.plate_height_m
      << YAML::Key << "pose_label" << YAML::Value << config.pose.label
      << YAML::Key << "aim" << YAML::Value << aim_config_yaml(config.aim)
      << YAML::Key << "max_reprojection_error_px" << YAML::Value << config.pose.max_reprojection_error_px
      << YAML::EndMap;
  if (!out.good()) throw std::runtime_error("cannot serialize effective configuration");
  return std::string(out.c_str()) + '\n';
}
void save_config(const PerceptionConfig& config, const std::string& path) {
  std::ofstream out(path);
  out << config_yaml(config);
  out.flush();
  if (!out) throw std::runtime_error("cannot write effective configuration: " + path);
}
}  // namespace bof
