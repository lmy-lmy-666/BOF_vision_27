// YOLOv8 / YOLO11 keypoint decoding and the tiny_resnet patch adapted from TongjiSuperPower/sp_vision_25
// tasks/auto_aim/yolos/yolov8.cpp, yolo11.cpp, armor.hpp and yolov8.cpp get_pattern (MIT).
// See third_party/licenses/TongjiSuperPower-MIT.txt.
#include "bof_vision/detector_factory.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace bof {
namespace {
enum ModelColor {blue, red, extinguished, purple};
struct ClassInfo {ModelColor color; ArmorLabel label; PlateSize size;};
// Tongji armor_properties, index order of the 38-class YOLO11 head.
std::vector<ClassInfo> yolo11_classes() {
  std::vector<ClassInfo> t;
  auto triple=[&](ArmorLabel l,PlateSize s){for(auto c:{blue,red,extinguished}) t.push_back({c,l,s});};
  // Only explicit size classes become hints. Upstream lists hero/sentry/engineer/outpost as "small"
  // although hero plates are large; leave those unknown for the geometry ratio check.
  triple(ArmorLabel::sentry,PlateSize::unknown); triple(ArmorLabel::hero,PlateSize::unknown);
  triple(ArmorLabel::engineer,PlateSize::unknown); triple(ArmorLabel::infantry3,PlateSize::small);
  triple(ArmorLabel::infantry4,PlateSize::small); triple(ArmorLabel::infantry5,PlateSize::small);
  triple(ArmorLabel::outpost,PlateSize::unknown);
  for(auto c:{blue,red,extinguished,purple}) t.push_back({c,ArmorLabel::base,PlateSize::large});
  for(auto c:{blue,red,extinguished,purple}) t.push_back({c,ArmorLabel::base,PlateSize::small});
  triple(ArmorLabel::infantry3,PlateSize::large); triple(ArmorLabel::infantry4,PlateSize::large);
  triple(ArmorLabel::infantry5,PlateSize::large);
  return t;
}
// Tongji sort_keypoints: two highest points are the top edge, then left/right by x.
std::array<cv::Point2f,4> sort_corners(std::array<cv::Point2f,4> p) {
  std::sort(p.begin(),p.end(),[](const auto& a,const auto& b){return a.y<b.y;});
  if(p[0].x>p[1].x) std::swap(p[0],p[1]);
  if(p[2].x>p[3].x) std::swap(p[2],p[3]);
  return {p[0],p[1],p[3],p[2]};
}
}
ArmorLabel tongji_number_label(int index) {
  static const ArmorLabel labels[]={ArmorLabel::hero,ArmorLabel::engineer,ArmorLabel::infantry3,ArmorLabel::infantry4,
    ArmorLabel::infantry5,ArmorLabel::sentry,ArmorLabel::outpost,ArmorLabel::base};
  return index>=0 && index<8?labels[index]:ArmorLabel::unknown;
}
std::vector<Candidate> decode_keypoint_yolo(const cv::Mat& rows,KeypointModel model,double scale,
  cv::Size original_size,Color enemy,float threshold,float nms) {
  static const auto table=yolo11_classes();
  const int classes=model==KeypointModel::tongji_yolov8?2:static_cast<int>(table.size());
  if(rows.type()!=CV_32FC1 || rows.dims!=2 || rows.cols!=4+classes+8)
    throw std::invalid_argument("keypoint YOLO output must be N x (4+classes+8) float32");
  if(!std::isfinite(scale) || scale<=0 || original_size.width<=0 || original_size.height<=0 ||
     !std::isfinite(threshold) || threshold<=0 || threshold>=1 || !std::isfinite(nms) || nms<=0 || nms>=1)
    throw std::invalid_argument("invalid decoder dimensions or thresholds");
  const ModelColor wanted=enemy==Color::red?red:blue;
  std::vector<Candidate> all; std::vector<cv::Rect2d> boxes; std::vector<float> scores;
  for(int r=0;r<rows.rows;++r) {
    const float* p=rows.ptr<float>(r);
    if(!std::all_of(p,p+rows.cols,[](float x){return std::isfinite(x);})) continue;
    const int best=static_cast<int>(std::max_element(p+4,p+4+classes)-(p+4));
    const float score=p[4+best];
    if(score<threshold) continue;
    Candidate c; c.score=c.confidence=score; c.raw_class=best;
    if(model==KeypointModel::tongji_yolov8) {
      if((best==0?blue:red)!=wanted) continue;   // number comes from the classifier
    } else {
      const auto& info=table[best];
      if(info.color!=wanted) continue;
      c.label=info.label; c.size_hint=info.size;
    }
    std::array<cv::Point2f,4> points;
    for(int i=0;i<4;++i) points[i]={static_cast<float>(p[4+classes+2*i]/scale),static_cast<float>(p[4+classes+2*i+1]/scale)};
    c.corners=sort_corners(points);
    std::vector<cv::Point2f> polygon(c.corners.begin(),c.corners.end());
    if(!cv::isContourConvex(polygon) || cv::contourArea(polygon)<4.0) continue;
    const auto center=c.center();
    if(center.x<0 || center.y<0 || center.x>=original_size.width || center.y>=original_size.height) continue;
    boxes.emplace_back((p[0]-0.5*p[2])/scale,(p[1]-0.5*p[3])/scale,p[2]/scale,p[3]/scale);
    scores.push_back(score); all.push_back(c);
  }
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes,scores,threshold,nms,indices);
  std::vector<Candidate> result;
  for(int i:indices) result.push_back(all.at(i));
  return result;
}
cv::Mat classifier_patch(const cv::Mat& bgr,const Candidate& c) {
  // Extend each light bar 1.125x its length above and below its midpoint, as Tongji get_pattern.
  const auto& p=c.corners;   // TL, TR, BR, BL
  const cv::Point2f left=(p[0]+p[3])/2, right=(p[1]+p[2])/2, left_dir=p[3]-p[0], right_dir=p[2]-p[1];
  const cv::Point2f tl=left-left_dir*1.125F, bl=left+left_dir*1.125F, tr=right-right_dir*1.125F, br=right+right_dir*1.125F;
  const int x0=std::max(0,static_cast<int>(std::min(tl.x,bl.x))), y0=std::max(0,static_cast<int>(std::min(tl.y,tr.y)));
  const int x1=std::min(bgr.cols,static_cast<int>(std::max(tr.x,br.x))), y1=std::min(bgr.rows,static_cast<int>(std::max(bl.y,br.y)));
  if(x1<=x0 || y1<=y0) return {};
  cv::Mat gray; cv::cvtColor(bgr(cv::Rect(x0,y0,x1-x0,y1-y0)),gray,cv::COLOR_BGR2GRAY);
  const double s=std::min(32.0/gray.cols,32.0/gray.rows);
  const int w=static_cast<int>(gray.cols*s), h=static_cast<int>(gray.rows*s);
  if(w<1 || h<1) return {};
  cv::Mat input(32,32,CV_8UC1,cv::Scalar(0));
  cv::resize(gray,input(cv::Rect(0,0,w,h)),{w,h});
  return input;
}
}  // namespace bof
