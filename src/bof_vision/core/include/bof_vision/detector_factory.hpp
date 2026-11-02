#pragma once
#include "bof_vision/core.hpp"
#include <memory>
#include <string>

namespace bof {
struct DetectorConfig {
  std::string backend = "light_pair";
  Color enemy_color = Color::red;
  int color_threshold = 80;
  std::string model_path;
  std::string device = "CPU";
  float confidence_threshold = 0.7F;
  float nms_threshold = 0.3F;
  // tongji_yolov8 only: tiny_resnet number classifier and its minimum softmax confidence.
  std::string classifier_path;
  float classifier_confidence = 0.8F;
};
std::unique_ptr<Detector> make_detector(const DetectorConfig& config);
// Contract: [N,22] float32, LT/LB/RB/RT, objectness logit, 4 color logits,
// 9 class logits; top-left letterbox (black), 640x640, RGB/255.
std::vector<Candidate> decode_tongji_yolov5(const cv::Mat& rows, double scale,
  cv::Size original_size, Color enemy, float confidence_threshold, float nms_threshold);
enum class FourPointLayout { tongji_yolov5, shenzhen };
std::vector<Candidate> decode_four_point22(const cv::Mat& rows, double scale,
  cv::Size original_size, Color enemy, float confidence_threshold, float nms_threshold,
  FourPointLayout layout);
// Tongji sp_vision_25 keypoint YOLO heads, output transposed to N x (4+K+8):
// cx,cy,w,h, K class scores (already sigmoid), 4 keypoints. Letterbox scale as for YOLOv5.
//   tongji_yolov8: K=2, class = colour (0 blue, 1 red); number from the classifier.
//   tongji_yolo11: K=38, class = (colour, number, plate size) table from armor.hpp.
enum class KeypointModel { tongji_yolov8, tongji_yolo11 };
std::vector<Candidate> decode_keypoint_yolo(const cv::Mat& rows, KeypointModel model, double scale,
  cv::Size original_size, Color enemy, float confidence_threshold, float nms_threshold);
// tiny_resnet input: gray 32x32 top-left letterbox of the plate region extended to the light bars.
cv::Mat classifier_patch(const cv::Mat& bgr, const Candidate& candidate);
// Classifier class index (one..five, sentry, outpost, base, not_armor) -> label; unknown for not_armor.
ArmorLabel tongji_number_label(int index);
}  // namespace bof
