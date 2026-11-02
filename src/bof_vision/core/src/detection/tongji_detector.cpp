// Preprocessing adapted from TongjiSuperPower/sp_vision_25 (MIT): yolov5.cpp, yolov8.cpp, yolo11.cpp, classifier.cpp.
// Copyright (c) 2025 TongjiSuperPower. See third_party/licenses/TongjiSuperPower-MIT.txt.
#include "bof_vision/detector_factory.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <opencv2/imgproc.hpp>
#include <openvino/openvino.hpp>
#include <openvino/core/preprocess/pre_post_process.hpp>
#include <opencv2/dnn.hpp>

namespace bof {
class TongjiDetector final : public Detector {
public:
  explicit TongjiDetector(const DetectorConfig& config) : config_(config) {
    if (config.model_path.empty()) throw std::invalid_argument("model_path is required for neural detector");
    if (!std::isfinite(config.confidence_threshold) || config.confidence_threshold<=0 || config.confidence_threshold>=1 ||
        !std::isfinite(config.nms_threshold) || config.nms_threshold<=0 || config.nms_threshold>=1)
      throw std::invalid_argument("detector thresholds must be in (0,1)");
    // Output contracts: four-point 1xNx22 (yolov5/shenzhen); keypoint heads 1x(4+K+8)xN, transposed at decode.
    if (config.backend=="tongji_yolov8") {size_=416; channels_=14; keypoint_=true; model_=KeypointModel::tongji_yolov8;}
    else if (config.backend=="tongji_yolo11") {size_=640; channels_=50; keypoint_=true; model_=KeypointModel::tongji_yolo11;}
    auto model = core_.read_model(config.model_path);
    if (model->inputs().size()!=1 || model->outputs().size()!=1 ||
        model->input().get_partial_shape()!=ov::PartialShape{1,3,size_,size_})
      throw std::invalid_argument("expected one 1x3x"+std::to_string(size_)+"x"+std::to_string(size_)+" input and one output for "+config.backend);
    auto shape = model->output().get_partial_shape();
    const int channel_axis=keypoint_?1:2;
    if (!shape.is_static() || shape.rank().get_length()!=3 || shape[0].get_length()!=1 || shape[channel_axis].get_length()!=channels_)
      throw std::invalid_argument("model output does not match the "+config.backend+" contract");
    if (config.backend=="tongji_yolov8") {
      // The YOLOv8 head only gives colour; Tongji tiny_resnet classifies the number.
      if (config.classifier_path.empty()) throw std::invalid_argument("tongji_yolov8 requires classifier_path (tiny_resnet.onnx)");
      classifier_=cv::dnn::readNetFromONNX(config.classifier_path);
      if (classifier_.empty()) throw std::invalid_argument("cannot read classifier "+config.classifier_path);
    }
    ov::preprocess::PrePostProcessor ppp(model);
    ppp.input().tensor().set_element_type(ov::element::u8).set_shape({1,size_,size_,3})
      .set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::BGR);
    ppp.input().model().set_layout("NCHW");
    ppp.input().preprocess().convert_element_type(ov::element::f32)
      .convert_color(ov::preprocess::ColorFormat::RGB).scale(255.F);
    ppp.output().postprocess().convert_element_type(ov::element::f32);
    compiled_ = core_.compile_model(ppp.build(),config.device,
      ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
    request_ = compiled_.create_infer_request();
  }
  void set_parameters(Color color,int,float confidence,float nms) override {
    if(!std::isfinite(confidence) || confidence<=0 || confidence>=1 || !std::isfinite(nms) || nms<=0 || nms>=1)
      throw std::invalid_argument("detector thresholds must be in (0,1)");
    config_.enemy_color=color; config_.confidence_threshold=confidence; config_.nms_threshold=nms;
  }
  std::vector<Candidate> detect(const cv::Mat& bgr) const override {
    if (bgr.empty() || bgr.type()!=CV_8UC3) throw std::invalid_argument("expected nonempty BGR8");
    const double scale = std::min(double(size_)/bgr.cols,double(size_)/bgr.rows);
    int width = static_cast<int>(bgr.cols*scale), height=static_cast<int>(bgr.rows*scale);
    if (width<1 || height<1) throw std::invalid_argument("unsupported extreme image aspect ratio");
    cv::Mat input(size_,size_,CV_8UC3,cv::Scalar(0,0,0));
    cv::resize(bgr,input(cv::Rect(0,0,width,height)),{width,height});
    request_.set_input_tensor(ov::Tensor(ov::element::u8,{1,std::size_t(size_),std::size_t(size_),3},input.data));
    request_.infer();
    auto output = request_.get_output_tensor();
    const auto shape=output.get_shape();
    const std::size_t channel_axis=keypoint_?1:2;
    if (shape.size()!=3 || shape[0]!=1 || shape[channel_axis]!=std::size_t(channels_) || output.get_element_type()!=ov::element::f32)
      throw std::runtime_error("unexpected runtime model output");
    if (!keypoint_) {
      cv::Mat rows(static_cast<int>(shape[1]),22,CV_32F,output.data<float>());
      return decode_four_point22(rows,scale,bgr.size(),config_.enemy_color,
        config_.confidence_threshold,config_.nms_threshold,
        config_.backend=="tongji_yolov5" ? FourPointLayout::tongji_yolov5 : FourPointLayout::shenzhen);
    }
    cv::Mat rows;
    cv::transpose(cv::Mat(channels_,static_cast<int>(shape[2]),CV_32F,output.data<float>()),rows);
    auto candidates=decode_keypoint_yolo(rows,model_,scale,bgr.size(),config_.enemy_color,
      config_.confidence_threshold,config_.nms_threshold);
    if (model_==KeypointModel::tongji_yolov8) classify(bgr,candidates);
    return candidates;
  }
private:
  // Tongji Classifier::classify: softmax over one..five, sentry, outpost, base, not_armor.
  void classify(const cv::Mat& bgr,std::vector<Candidate>& candidates) const {
    std::vector<Candidate> kept;
    for (auto& c:candidates) {
      const cv::Mat patch=classifier_patch(bgr,c);
      if (patch.empty()) continue;
      classifier_.setInput(cv::dnn::blobFromImage(patch,1.0/255.0));
      cv::Mat out=classifier_.forward().reshape(1,1);
      if (out.cols!=9) throw std::runtime_error("classifier must output 9 classes");
      double top; cv::Point index;
      cv::exp(out-*std::max_element(out.begin<float>(),out.end<float>()),out); out/=cv::sum(out)[0];
      cv::minMaxLoc(out,nullptr,&top,nullptr,&index);
      if (index.x==8 || top<config_.classifier_confidence) continue;
      c.label=tongji_number_label(index.x); c.raw_class=index.x;
      kept.push_back(c);
    }
    candidates=std::move(kept);
  }
  int size_=640, channels_=22;
  bool keypoint_=false;
  KeypointModel model_=KeypointModel::tongji_yolo11;
  mutable cv::dnn::Net classifier_;
  DetectorConfig config_;
  ov::Core core_;
  ov::CompiledModel compiled_;
  mutable ov::InferRequest request_;  // Single consumer, synchronous API; not thread-safe.
};
std::unique_ptr<Detector> make_tongji_detector(const DetectorConfig& config) {
  return std::make_unique<TongjiDetector>(config);
}
}  // namespace bof
