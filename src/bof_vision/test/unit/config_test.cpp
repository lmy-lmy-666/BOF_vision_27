#include "bof_vision/config.hpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <limits>
#include <cstdlib>

namespace {
class Configuration : public ::testing::Test {
protected:
  std::filesystem::path directory;
  void SetUp() override {
    std::string pattern = (std::filesystem::temp_directory_path()/"bof-config-XXXXXX").string();
    auto* created = mkdtemp(pattern.data());
    ASSERT_NE(created, nullptr);
    directory = created;
  }
  void TearDown() override { std::filesystem::remove_all(directory); }
  std::string write(const std::string& name, const std::string& text) {
    auto path = directory/name;
    std::filesystem::create_directories(path.parent_path());
    std::ofstream(path) << text;
    return path.string();
  }
};
TEST_F(Configuration, InheritanceAndPathsBelongToDeclaringFile) {
  write("base/model.onnx", "fixture, not an inference model");
  write("base/base.yaml", "detector_backend: shenzhen_0526\nmodel_path: model.onnx\nenemy_color: blue\ntrack_timeout_s: 0.5\n");
  auto path = write("profile.yaml", "extends: base/base.yaml\ntrack_timeout_s: 0.8\n");
  auto config = bof::load_config(path);
  bof::validate_config(config);
  EXPECT_EQ(config.detector.enemy_color, bof::Color::blue);
  EXPECT_EQ(config.detector.model_path, (directory/"base/model.onnx").string());
  EXPECT_DOUBLE_EQ(config.track_timeout_s, 0.8);
  auto override = bof::load_config(write("override.yaml", "extends: base/base.yaml\nmodel_path: another.onnx\n"));
  EXPECT_EQ(override.detector.model_path, (directory/"another.onnx").string());
}
TEST_F(Configuration, RejectsTyposDuplicatesTypesAndCycles) {
  for (const std::string text : {"confidence_thresold: 0.5\n", "enemy_color: red\nenemy_color: blue\n",
       "color_threshold: 2.5\n", "publish_debug_image: maybe\n", "model_path: []\n",
       "schema_version: 2\n", "enemy_color: green\n", "extends: ''\n", "- not_a_mapping\n"})
    EXPECT_THROW(bof::load_config(write("bad.yaml", text)), std::exception) << text;
  write("a.yaml", "extends: b.yaml\n");
  write("b.yaml", "extends: a.yaml\n");
  EXPECT_THROW(bof::load_config((directory/"a.yaml").string()), std::exception);
  EXPECT_THROW(bof::load_config((directory/"missing.yaml").string()), std::exception);
}
TEST_F(Configuration, EffectiveValidationAndSnapshotRoundTrip) {
  auto config = bof::load_config(write("input.yaml", "enemy_color: blue\nconfidence_threshold: .nan\n"));
  EXPECT_THROW(bof::validate_config(config), std::invalid_argument);
  config.detector.confidence_threshold = 0.6F;
  config.detector.nms_threshold = 1.1F;
  EXPECT_THROW(bof::validate_config(config), std::invalid_argument);
  config.detector.nms_threshold = 0.4F;
  config.track_timeout_s = 0;
  EXPECT_THROW(bof::validate_config(config), std::invalid_argument);
  config.track_timeout_s = 0.4;
  config.association_gate_px = std::numeric_limits<double>::infinity();
  EXPECT_THROW(bof::validate_config(config), std::invalid_argument);
  config.association_gate_px = 45;
  config.detector.backend = "unsupported";
  EXPECT_THROW(bof::validate_config(config), std::invalid_argument);
  config.detector.backend = "shenzhen_0526";
  EXPECT_THROW(bof::validate_config(config), std::invalid_argument);
  config.detector.backend = "light_pair";
  bof::validate_config(config);
  auto snapshot = (directory/"effective.yaml").string();
  bof::save_config(config, snapshot);
  EXPECT_EQ(bof::config_yaml(bof::load_config(snapshot)), bof::config_yaml(config));
  EXPECT_THROW(bof::save_config(config, (directory/"missing/out.yaml").string()), std::runtime_error);
}
TEST_F(Configuration, LegacyRosYamlRemainsReadable) {
  auto config = bof::load_config(write("legacy.yaml", "pipeline:\n  ros__parameters:\n    enemy_color: blue\n    association_gate_px: 45.0\n"));
  EXPECT_EQ(config.detector.enemy_color, bof::Color::blue);
  EXPECT_DOUBLE_EQ(config.association_gate_px, 45);
  EXPECT_THROW(bof::load_config(write("bad_legacy.yaml", "pipeline:\n  ros__parameters:\n    typo: 1\n")), std::exception);
}
TEST_F(Configuration, NamedModelsAndRuntimePathsRoundTrip) {
  auto input=write("base/sentry.yaml",
    "yolo_name: geometry\nyolo_models:\n"
    "  geometry: {backend: light_pair, confidence: 0.6, nms: 0.2}\n"
    "  unavailable: {backend: shenzhen_0526, model_path: missing.onnx}\n"
    "live: {source: video, video_path: input.avi, exposure_us: 1234, gain: 2, record: true}\n");
  auto config=bof::load_config(input); bof::validate_config(config);
  EXPECT_EQ(config.detector.backend,"light_pair");
  EXPECT_FLOAT_EQ(config.detector.confidence_threshold,0.6F);
  EXPECT_EQ(config.live.video_path,(directory/"base/input.avi").string());
  EXPECT_DOUBLE_EQ(config.live.exposure_us,1234);
  EXPECT_TRUE(config.live.record);
  // A child overriding only a runtime parameter must not erase inherited tuning.
  write("base/tuned.yaml","extends: sentry.yaml\nconfidence_threshold: 0.45\n");
  auto inherited=bof::load_config(write("child.yaml","extends: base/tuned.yaml\nlive: {gain: 5}\n"));
  EXPECT_FLOAT_EQ(inherited.detector.confidence_threshold,0.45F);
  EXPECT_DOUBLE_EQ(inherited.live.exposure_us,1234);
  EXPECT_DOUBLE_EQ(inherited.live.gain,5);
  const auto snapshot=write("snapshot.yaml",bof::config_yaml(inherited));
  EXPECT_EQ(bof::config_yaml(bof::load_config(snapshot)),bof::config_yaml(inherited));
  EXPECT_THROW(bof::select_yolo(config,"typo"),std::invalid_argument);
  bof::select_yolo(config,"unavailable");
  EXPECT_THROW(bof::validate_config(config),std::invalid_argument);
}
}  // namespace
