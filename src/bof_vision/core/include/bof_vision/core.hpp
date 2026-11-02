#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

namespace bof {
// Model-independent semantic ids. These deliberately do not use upstream enum ordinals.
enum class ArmorLabel : int { unknown=-1, sentry=0, hero=1, engineer=2,
  infantry3=3, infantry4=4, infantry5=5, outpost=6, base=7 };
enum class Color { red, blue };
enum class PlateSize : int { unknown=0, small=1, large=2 };
const char* plate_size_name(PlateSize size);
struct Candidate {
  std::array<cv::Point2f, 4> corners;  // TL, TR, BR, BL
  float score = 0;  // Ranking score. See confidence to distinguish neural output.
  ArmorLabel label = ArmorLabel::unknown;
  float confidence = -1;  // -1 = unclassified geometric candidate; otherwise model objectness.
  int raw_class = -1;
  PlateSize size_hint = PlateSize::unknown;  // Only explicit model semantics, never guessed from identity.
  cv::Point2f center() const;
};
class Detector {
public:
  virtual ~Detector() = default;
  virtual std::vector<Candidate> detect(const cv::Mat& bgr) const = 0;
  virtual void set_parameters(Color color, int threshold, float confidence, float nms) = 0;
};
// Integration baseline only: no number classification or target identity.
class LightPairDetector final : public Detector {
public:
  explicit LightPairDetector(Color color, int threshold = 80);
  std::vector<Candidate> detect(const cv::Mat& bgr) const override;
  void set_parameters(Color color, int threshold, float confidence, float nms) override;
private:
  Color color_;
  int threshold_;
};
struct PixelTrack {
  bool valid = false;
  std::uint64_t id = 0;
  cv::Point2f center{};
  ArmorLabel label = ArmorLabel::unknown;
  bool observed = false;
  int candidate_index = -1;
  ArmorLabel observed_label = ArmorLabel::unknown;
  std::string status = "lost";
  double association_error_px = -1;
};
struct AssociationConfig {
  int confirm_frames = 3;
  int label_confirm_frames = 3;
  double ambiguity_px = 12.0;
};
// One image-space target; does not compensate camera motion or rotation.
class PixelTracker {
public:
  PixelTracker(double timeout_s = 0.2, double gate_px = 100.0,
               const AssociationConfig& config = {});
  PixelTrack update(const std::vector<Candidate>& detections, double stamp_s);
  void reset();
private:
  double timeout_, gate_;
  AssociationConfig config_;
  cv::KalmanFilter filter_;
  std::optional<double> last_stamp_, last_seen_;
  bool active_ = false, confirmed_ = false;
  int hits_ = 0, pending_hits_ = 0;
  ArmorLabel pending_label_ = ArmorLabel::unknown;
  ArmorLabel label_ = ArmorLabel::unknown;
  std::uint64_t id_ = 0;
  void start(const Candidate& candidate, double stamp, int hits=1);
};
cv::Mat synthetic_frame(int index, Color color);
}  // namespace bof
