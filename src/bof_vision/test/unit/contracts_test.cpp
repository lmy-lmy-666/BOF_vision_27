#include "bof_vision/detector_factory.hpp"
#include "bof_vision/pipeline.hpp"
#include "bof_vision/latest_buffer.hpp"
#include "bof_vision/orientation_buffer.hpp"
#include <gtest/gtest.h>
#include <future>
#include <limits>

namespace {
cv::Mat model_row(int color=1,int label=6) {
  cv::Mat row(1,22,CV_32F,cv::Scalar(-8));
  const float values[]={100,100,100,140,180,140,180,100};
  std::copy(values,values+8,row.ptr<float>());
  row.at<float>(0,8)=8;
  row.at<float>(0,9+color)=8;
  row.at<float>(0,13+label)=8;
  return row;
}
}
TEST(ModelContract, ColorsClassesAndCornerOrder) {
  auto result=bof::decode_tongji_yolov5(model_row(),0.5,{1280,720},bof::Color::red,0.7,0.3);
  ASSERT_EQ(result.size(),1U);
  EXPECT_EQ(result[0].label,bof::ArmorLabel::outpost);
  EXPECT_EQ(result[0].corners[0],cv::Point2f(200,200));
  EXPECT_EQ(result[0].corners[1],cv::Point2f(360,200));
  EXPECT_EQ(result[0].corners[2],cv::Point2f(360,280));
  EXPECT_EQ(result[0].corners[3],cv::Point2f(200,280));
  EXPECT_TRUE(bof::decode_tongji_yolov5(model_row(0),1,{640,640},bof::Color::red,0.7,0.3).empty());
  EXPECT_EQ(bof::decode_tongji_yolov5(model_row(0,3),1,{640,640},bof::Color::blue,0.7,0.3)[0].label,bof::ArmorLabel::infantry3);
  EXPECT_TRUE(bof::decode_tongji_yolov5(model_row(1,8),1,{640,640},bof::Color::red,0.7,0.3).empty());
}
TEST(ModelContract, NmsBadGeometryAndWrongModel) {
  cv::Mat rows; cv::vconcat(model_row(),model_row(),rows);
  EXPECT_EQ(bof::decode_tongji_yolov5(rows,1,{640,640},bof::Color::red,0.7,0.3).size(),1U);
  auto bad=model_row(); bad.at<float>(0,0)=std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(bof::decode_tongji_yolov5(bad,1,{640,640},bof::Color::red,0.7,0.3).empty());
  EXPECT_THROW(bof::decode_tongji_yolov5(cv::Mat::zeros(1,21,CV_32F),1,{640,640},bof::Color::red,0.7,0.3),std::invalid_argument);
}
TEST(ModelContract, SameShapeDoesNotMeanSameClassLayout) {
  auto rows=model_row(1,8);
  EXPECT_TRUE(bof::decode_tongji_yolov5(rows,1,{640,640},bof::Color::red,0.7,0.3).empty());
  auto result=bof::decode_four_point22(rows,1,{640,640},bof::Color::red,0.7,0.3,bof::FourPointLayout::shenzhen);
  ASSERT_EQ(result.size(),1U);
  EXPECT_EQ(result[0].label,bof::ArmorLabel::base);
}
// Keypoint head row: cx,cy,w,h, K scores, 4 unordered keypoints.
cv::Mat keypoint_row(int classes,int best,float score=0.9F) {
  cv::Mat row=cv::Mat::zeros(1,4+classes+8,CV_32F);
  float* p=row.ptr<float>();
  p[0]=280; p[1]=240; p[2]=160; p[3]=80; p[4+best]=score;
  const float points[]={360,280, 200,200, 200,280, 360,200};   // BR, TL, BL, TR: decoder must sort
  std::copy(points,points+8,p+4+classes);
  return row;
}
TEST(ModelContract, TongjiYolo11ClassTableAndCornerSort) {
  // Class 19 = red outpost, 22 = red large base, 31 = blue large infantry4 (Tongji armor_properties).
  auto result=bof::decode_keypoint_yolo(keypoint_row(38,19),bof::KeypointModel::tongji_yolo11,1,{640,640},bof::Color::red,0.7,0.3);
  ASSERT_EQ(result.size(),1U);
  EXPECT_EQ(result[0].label,bof::ArmorLabel::outpost); EXPECT_EQ(result[0].size_hint,bof::PlateSize::unknown);
  EXPECT_EQ(result[0].corners[0],cv::Point2f(200,200)); EXPECT_EQ(result[0].corners[1],cv::Point2f(360,200));
  EXPECT_EQ(result[0].corners[2],cv::Point2f(360,280)); EXPECT_EQ(result[0].corners[3],cv::Point2f(200,280));
  auto base=bof::decode_keypoint_yolo(keypoint_row(38,22),bof::KeypointModel::tongji_yolo11,1,{640,640},bof::Color::red,0.7,0.3);
  ASSERT_EQ(base.size(),1U); EXPECT_EQ(base[0].label,bof::ArmorLabel::base); EXPECT_EQ(base[0].size_hint,bof::PlateSize::large);
  auto balance=bof::decode_keypoint_yolo(keypoint_row(38,32),bof::KeypointModel::tongji_yolo11,1,{640,640},bof::Color::blue,0.7,0.3);
  ASSERT_EQ(balance.size(),1U); EXPECT_EQ(balance[0].label,bof::ArmorLabel::infantry4); EXPECT_EQ(balance[0].size_hint,bof::PlateSize::large);
  EXPECT_TRUE(bof::decode_keypoint_yolo(keypoint_row(38,19),bof::KeypointModel::tongji_yolo11,1,{640,640},bof::Color::blue,0.7,0.3).empty());
  EXPECT_TRUE(bof::decode_keypoint_yolo(keypoint_row(38,20),bof::KeypointModel::tongji_yolo11,1,{640,640},bof::Color::red,0.7,0.3).empty());
  EXPECT_THROW(bof::decode_keypoint_yolo(keypoint_row(2,1),bof::KeypointModel::tongji_yolo11,1,{640,640},bof::Color::red,0.7,0.3),std::invalid_argument);
}
TEST(ModelContract, TongjiYolov8ColourOnlyAndClassifierPatch) {
  auto red=bof::decode_keypoint_yolo(keypoint_row(2,1),bof::KeypointModel::tongji_yolov8,0.5,{1280,720},bof::Color::red,0.7,0.3);
  ASSERT_EQ(red.size(),1U); EXPECT_EQ(red[0].label,bof::ArmorLabel::unknown);
  EXPECT_EQ(red[0].corners[0],cv::Point2f(400,400));
  EXPECT_TRUE(bof::decode_keypoint_yolo(keypoint_row(2,0),bof::KeypointModel::tongji_yolov8,1,{640,640},bof::Color::red,0.7,0.3).empty());
  EXPECT_TRUE(bof::decode_keypoint_yolo(keypoint_row(2,1,0.5F),bof::KeypointModel::tongji_yolov8,1,{640,640},bof::Color::red,0.7,0.3).empty());
  cv::Mat image(480,640,CV_8UC3,cv::Scalar(10,20,30));
  auto patch=bof::classifier_patch(image,red[0]);
  EXPECT_EQ(patch.size(),cv::Size(32,32)); EXPECT_EQ(patch.type(),CV_8UC1);
  EXPECT_EQ(bof::tongji_number_label(0),bof::ArmorLabel::hero); EXPECT_EQ(bof::tongji_number_label(5),bof::ArmorLabel::sentry);
  EXPECT_EQ(bof::tongji_number_label(8),bof::ArmorLabel::unknown);
}
TEST(Pipeline, ResetOnResolutionAndClockChange) {
  bof::PerceptionPipeline pipeline(bof::DetectorConfig{});
  bof::Frame f{bof::synthetic_frame(0,bof::Color::red),1000000000,0};
  auto first=pipeline.process(f);
  ASSERT_TRUE(first.track.observed); EXPECT_FALSE(first.track.valid);
  EXPECT_EQ(first.track.status,"detecting");
  f.stamp_ns+=10000000; f.clock=bof::ClockDomain::host_receive;
  EXPECT_GT(pipeline.process(f).track.id,first.track.id);
  auto id=pipeline.process(f).track.id;
  f.bgr=cv::Mat(500,640,CV_8UC3,cv::Scalar(0,0,0));
  bof::synthetic_frame(0,bof::Color::red).copyTo(f.bgr(cv::Rect(0,0,640,480)));
  f.stamp_ns+=10000000;
  EXPECT_GT(pipeline.process(f).track.id,id);
}
TEST(Tracker, DoesNotAssociateDifferentSemanticTarget) {
  bof::PixelTracker tracker(0.2,100,{1,3,12});
  auto detections=bof::decode_tongji_yolov5(model_row(),1,{640,640},bof::Color::red,0.7,0.3);
  ASSERT_TRUE(tracker.update(detections,1).valid);
  detections[0].label=bof::ArmorLabel::infantry3;
  EXPECT_FALSE(tracker.update(detections,1.03).valid);
  EXPECT_TRUE(tracker.update(detections,1.3).valid);
}
TEST(LatestBuffer, OverwriteAndCloseWakeConsumer) {
  bof::LatestBuffer<int> buffer;
  buffer.push(1); buffer.push(2); buffer.push(3);
  EXPECT_EQ(buffer.overwritten(),2U);
  EXPECT_EQ(*buffer.wait_pop(),3);
  auto waiting=std::async(std::launch::async,[&]{return buffer.wait_pop();});
  buffer.close();
  ASSERT_EQ(waiting.wait_for(std::chrono::seconds(1)),std::future_status::ready);
  EXPECT_FALSE(waiting.get()); EXPECT_FALSE(buffer.push(4));
}
TEST(Orientation, InterpolationAndNoExtrapolation) {
  bof::OrientationBuffer buffer(bof::ClockDomain::exposure_synchronized,100);
  EXPECT_TRUE(buffer.push({100,Eigen::Quaterniond::Identity()}));
  EXPECT_TRUE(buffer.push({200,Eigen::Quaterniond(Eigen::AngleAxisd(1.0,Eigen::Vector3d::UnitZ()))}));
  auto q=buffer.at(150,bof::ClockDomain::exposure_synchronized);
  ASSERT_TRUE(q); EXPECT_NEAR(Eigen::AngleAxisd(*q).angle(),0.5,1e-9);
  EXPECT_FALSE(buffer.at(201,bof::ClockDomain::exposure_synchronized));
  EXPECT_FALSE(buffer.at(150,bof::ClockDomain::host_receive));
  EXPECT_FALSE(buffer.push({200,Eigen::Quaterniond::Identity()}));
  buffer.push({400,Eigen::Quaterniond::Identity()});
  EXPECT_FALSE(buffer.at(300,bof::ClockDomain::exposure_synchronized));
}
