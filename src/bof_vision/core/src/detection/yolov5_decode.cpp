// Adapted model decoding contract from TongjiSuperPower/sp_vision_25 (MIT).
// Copyright (c) 2025 TongjiSuperPower. See third_party/licenses/TongjiSuperPower-MIT.txt.
// Shenzhen layout cross-checked against Alliance models/shenzhen.hpp (Alliance-MIT.txt).
#include "bof_vision/detector_factory.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace bof {
std::vector<Candidate> decode_tongji_yolov5(const cv::Mat& rows, double scale,
  cv::Size original_size, Color enemy, float threshold, float nms) {
  return decode_four_point22(rows,scale,original_size,enemy,threshold,nms,FourPointLayout::tongji_yolov5);
}
std::vector<Candidate> decode_four_point22(const cv::Mat& rows, double scale,
  cv::Size original_size, Color enemy, float threshold, float nms, FourPointLayout layout) {
  if (rows.type()!=CV_32FC1 || rows.cols!=22 || rows.dims!=2)
    throw std::invalid_argument("Tongji YOLOv5 output must be N x 22 float32");
  if (!std::isfinite(scale) || scale<=0 || original_size.width<=0 || original_size.height<=0 ||
      !std::isfinite(threshold) || threshold<=0 || threshold>=1 || !std::isfinite(nms) || nms<=0 || nms>=1)
    throw std::invalid_argument("invalid decoder dimensions or thresholds");
  std::vector<Candidate> all;
  std::vector<cv::Rect2d> boxes;
  std::vector<float> scores;
  for (int r=0; r<rows.rows; ++r) {
    const float* p = rows.ptr<float>(r);
    if (!std::all_of(p,p+22,[](float x){return std::isfinite(x);})) continue;
    const float confidence = p[8]>=0 ? 1.F/(1.F+std::exp(-p[8])) : std::exp(p[8])/(1.F+std::exp(p[8]));
    if (confidence < threshold) continue;
    const int color = static_cast<int>(std::max_element(p+9,p+13)-(p+9));
    // Model color: 0 blue, 1 red, 2 dark, 3 mixed. Never cast to our Color enum.
    if (color != (enemy == Color::red ? 1 : 0)) continue;
    const int label = static_cast<int>(std::max_element(p+13,p+22)-(p+13));
    if (label == 8 && layout==FourPointLayout::tongji_yolov5) continue;
    Candidate c;
    c.score = c.confidence = confidence;
    // Shenzhen has two base classes; its class 8 is NOT the Tongji not-armor class.
    c.label = static_cast<ArmorLabel>(label==8 ? 7 : label);
    c.raw_class=label;
    if(layout==FourPointLayout::shenzhen && label>=7)
      c.size_hint=label==7 ? PlateSize::small : PlateSize::large;
    constexpr int order[] = {0,3,2,1};  // LT/LB/RB/RT -> TL/TR/BR/BL.
    for (int i=0; i<4; ++i) c.corners[i] = {static_cast<float>(p[2*order[i]]/scale),
                                                           static_cast<float>(p[2*order[i]+1]/scale)};
    std::vector<cv::Point2f> polygon(c.corners.begin(),c.corners.end());
    if (!cv::isContourConvex(polygon) || cv::contourArea(polygon)<4.0) continue;
    auto center = c.center();
    if (center.x<0 || center.y<0 || center.x>=original_size.width || center.y>=original_size.height) continue;
    float left=c.corners[0].x, right=left, top=c.corners[0].y, bottom=top;
    for (const auto& v:c.corners) { left=std::min(left,v.x); right=std::max(right,v.x);
      top=std::min(top,v.y); bottom=std::max(bottom,v.y); }
    boxes.emplace_back(left,top,right-left,bottom-top);
    scores.push_back(confidence); all.push_back(c);
  }
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes,scores,threshold,nms,indices);
  std::vector<Candidate> result;
  for (int i:indices) result.push_back(all.at(i));
  return result;
}
}  // namespace bof
