#include "bof_vision/core.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <opencv2/imgproc.hpp>

namespace bof {
void LightPairDetector::set_parameters(Color color,int threshold,float,float) {
  if(threshold<0 || threshold>255) throw std::invalid_argument("color threshold must be 0..255");
  color_=color; threshold_=threshold;
}
const char* plate_size_name(PlateSize size) {
  switch(size) { case PlateSize::small: return "small"; case PlateSize::large: return "large"; default: return "unknown"; }
}
cv::Point2f Candidate::center() const {
  cv::Point2f sum{};
  for (const auto& p : corners) sum += p;
  return sum * 0.25F;
}
LightPairDetector::LightPairDetector(Color color, int threshold)
: color_(color), threshold_(threshold) {
  if (threshold < 0 || threshold > 255) throw std::invalid_argument("threshold must be 0..255");
}
std::vector<Candidate> LightPairDetector::detect(const cv::Mat& bgr) const {
  if (bgr.empty() || bgr.type() != CV_8UC3) throw std::invalid_argument("expected nonempty BGR8");
  std::vector<cv::Mat> channels;
  cv::split(bgr, channels);
  cv::Mat mask;
  const int c = color_ == Color::red ? 2 : 0;
  cv::subtract(channels[c], channels[2-c], mask);
  cv::threshold(mask, mask, threshold_, 255, cv::THRESH_BINARY);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  std::vector<cv::Rect> lights;
  for (const auto& contour : contours) {
    auto r = cv::boundingRect(contour);
    if (r.height >= 6 && r.height >= 2*r.width && cv::contourArea(contour) >= 8) lights.push_back(r);
  }
  std::sort(lights.begin(), lights.end(), [](auto a, auto b) { return a.x < b.x; });
  std::vector<Candidate> result;
  // Bound pathological bright noise; this simple baseline is quadratic in light count.
  if (lights.size() > 128) return result;
  for (std::size_t i = 0; i < lights.size(); ++i) {
    for (std::size_t j = i+1; j < lights.size(); ++j) {
      const auto a = lights[i], b = lights[j];
      float h = (a.height+b.height)*0.5F;
      float ax = a.x+a.width*0.5F, bx = b.x+b.width*0.5F;
      float dy = std::abs((a.y+a.height*0.5F)-(b.y+b.height*0.5F));
      float ratio = (bx-ax)/h;
      if (ratio < 0.7F || ratio > 5.0F || dy/h > 0.4F ||
          static_cast<float>(std::min(a.height,b.height))/std::max(a.height,b.height) < 0.6F) continue;
      Candidate candidate;
      candidate.corners = {cv::Point2f(ax, a.y), cv::Point2f(bx, b.y),
                           cv::Point2f(bx, b.y+b.height), cv::Point2f(ax, a.y+a.height)};
      candidate.score = 1.0F-dy/h;
      result.push_back(candidate);
    }
  }
  return result;
}
cv::Mat synthetic_frame(int index, Color color) {
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0,0,0));
  int x = 280 + static_cast<int>(70*std::sin(index*0.04));
  cv::Scalar value = color == Color::red ? cv::Scalar(0,0,255) : cv::Scalar(255,0,0);
  cv::rectangle(image, {x,200,8,50}, value, cv::FILLED);
  cv::rectangle(image, {x+90,200,8,50}, value, cv::FILLED);
  return image;
}
}  // namespace bof
