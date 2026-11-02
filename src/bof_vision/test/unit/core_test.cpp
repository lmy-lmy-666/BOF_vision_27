#include "bof_vision/core.hpp"
#include <gtest/gtest.h>
#include <limits>

TEST(Detector, ColorAndBlank) {
  bof::LightPairDetector red(bof::Color::red), blue(bof::Color::blue);
  auto img = bof::synthetic_frame(0, bof::Color::red);
  auto ds = red.detect(img);
  ASSERT_EQ(ds.size(), 1U);
  EXPECT_NEAR(ds[0].center().x, 329.0, 1.0);
  EXPECT_TRUE(blue.detect(img).empty());
  EXPECT_TRUE(red.detect(cv::Mat::zeros(480,640,CV_8UC3)).empty());
  EXPECT_THROW(red.detect(cv::Mat{}), std::invalid_argument);
}
TEST(Tracker, MotionLossTimeoutAndRewind) {
  bof::LightPairDetector detector(bof::Color::red);
  bof::PixelTracker tracker(0.2,100,{1,3,12});
  auto a = detector.detect(bof::synthetic_frame(0,bof::Color::red));
  auto b = detector.detect(bof::synthetic_frame(1,bof::Color::red));
  auto first = tracker.update(a,1.0);
  ASSERT_TRUE(first.valid);
  EXPECT_EQ(tracker.update(b,1.03).id,first.id);
  EXPECT_FALSE(tracker.update({},1.06).valid);
  EXPECT_EQ(tracker.update(b,1.09).id,first.id);
  auto reacquired = tracker.update(b,1.4);
  EXPECT_GT(reacquired.id,first.id);
  EXPECT_GT(tracker.update(a,0.0).id,reacquired.id);
  EXPECT_FALSE(tracker.update(a,std::numeric_limits<double>::quiet_NaN()).valid);
}
TEST(Tracker, RejectJumpAndReacquireAfterTimeout) {
  bof::LightPairDetector detector(bof::Color::red);
  auto a = detector.detect(bof::synthetic_frame(0,bof::Color::red));
  auto far = a;
  for (auto& p : far[0].corners) p.x += 200;
  bof::PixelTracker tracker(0.2,100,{1,3,12});
  auto first = tracker.update(a,0.0);
  EXPECT_FALSE(tracker.update(far,0.05).valid);
  EXPECT_GT(tracker.update(far,0.3).id,first.id);
}

namespace {
bof::Candidate observation(float x,bof::ArmorLabel label=bof::ArmorLabel::infantry3) {
  bof::Candidate result;
  result.corners={cv::Point2f{x-10,80},{x+10,80},{x+10,120},{x-10,120}};
  result.score=1; result.label=label;
  return result;
}
}
TEST(Tracker, ConfirmationLabelHysteresisAndAmbiguity) {
  bof::PixelTracker tracker;
  auto original=observation(100), changed=observation(100,bof::ArmorLabel::infantry4);
  auto tentative=tracker.update({original},0);
  EXPECT_FALSE(tentative.valid); EXPECT_TRUE(tentative.observed);
  EXPECT_EQ(tentative.center,cv::Point2f{});
  EXPECT_EQ(tentative.status,"detecting");
  EXPECT_FALSE(tracker.update({original},.03).valid);
  auto confirmed=tracker.update({original},.06);
  ASSERT_TRUE(confirmed.valid);
  auto pending=tracker.update({changed},.09);
  EXPECT_FALSE(pending.valid); EXPECT_EQ(pending.id,confirmed.id);
  EXPECT_EQ(pending.label,original.label); EXPECT_EQ(pending.observed_label,changed.label);
  EXPECT_EQ(pending.status,"label_pending"); EXPECT_EQ(pending.center,cv::Point2f{});
  auto restored=tracker.update({original},.12);
  EXPECT_TRUE(restored.valid); EXPECT_EQ(restored.id,confirmed.id);
  EXPECT_FALSE(tracker.update({changed},.15).valid);
  EXPECT_FALSE(tracker.update({changed},.18).valid);
  auto switched=tracker.update({changed},.21);
  EXPECT_TRUE(switched.valid); EXPECT_GT(switched.id,confirmed.id);
  EXPECT_EQ(switched.label,changed.label);
  auto ambiguous=tracker.update({changed,observation(106,changed.label)},.24);
  EXPECT_FALSE(ambiguous.valid); EXPECT_FALSE(ambiguous.observed);
  EXPECT_EQ(ambiguous.status,"ambiguous"); EXPECT_EQ(ambiguous.candidate_index,-1);
  EXPECT_FALSE(tracker.update({},.27).valid);
  EXPECT_EQ(tracker.update({changed},.30).id,switched.id);
  auto expired=tracker.update({changed},.60);
  EXPECT_FALSE(expired.valid); EXPECT_GT(expired.id,switched.id);
}
TEST(Tracker, MotionPredictionAssociatesCurrentMeasurementAfterGap) {
  bof::PixelTracker tracker(.2,40,{3,3,5});
  tracker.update({observation(100)},0);
  tracker.update({observation(120)},.03);
  auto first=tracker.update({observation(140)},.06);
  ASSERT_TRUE(first.valid);
  auto moving=tracker.update({observation(142),observation(160)},.09);
  ASSERT_TRUE(moving.valid); EXPECT_EQ(moving.candidate_index,1);
  EXPECT_FLOAT_EQ(moving.center.x,160); EXPECT_EQ(moving.id,first.id);
  auto missing=tracker.update({},.12);
  EXPECT_FALSE(missing.valid); EXPECT_EQ(missing.center,cv::Point2f{});
  auto recovered=tracker.update({observation(200),observation(160)},.15);
  ASSERT_TRUE(recovered.valid); EXPECT_EQ(recovered.candidate_index,0);
  EXPECT_FLOAT_EQ(recovered.center.x,200); EXPECT_EQ(recovered.id,first.id);
}
