#include "bof_vision/detector_factory.hpp"
#include <stdexcept>

namespace bof {
#ifdef BOF_WITH_OPENVINO
std::unique_ptr<Detector> make_tongji_detector(const DetectorConfig& config);
#endif
std::unique_ptr<Detector> make_detector(const DetectorConfig& config) {
  if (config.backend == "light_pair")
    return std::make_unique<LightPairDetector>(config.enemy_color, config.color_threshold);
  if (config.backend == "tongji_yolov5" || config.backend == "tongji_yolov8" || config.backend == "tongji_yolo11" ||
      config.backend == "shenzhen_0708" || config.backend == "shenzhen_0526") {
#ifdef BOF_WITH_OPENVINO
    return make_tongji_detector(config);
#else
    throw std::invalid_argument("neural detector needs a build with BOF_WITH_OPENVINO=ON");
#endif
  }
  throw std::invalid_argument("unknown detector backend: " + config.backend);
}
}  // namespace bof
