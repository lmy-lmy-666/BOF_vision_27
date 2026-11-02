// Ratio-based size selection adapted from RPS TraditionalDetector::isArmor (MIT).
// Copyright (c) 2022 ChenJun
// Copyright (c) 2026 RPSvision DingZiHan
// Copyright (c) 2026 RPSvision WangXi
// See third_party/licenses/RPS-MIT.txt. We retain overlapping ranges as ambiguous
// instead of inventing the upstream number-classifier disambiguation step.
#include "bof_vision/geometry.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <set>
#include <stdexcept>

namespace bof {
namespace {
void keys(const YAML::Node& node,const std::set<std::string>& allowed) {
  if(!node.IsMap()) throw std::invalid_argument("geometry: expected mapping");
  std::set<std::string> seen;
  for(const auto& item:node) {
    const auto key=item.first.as<std::string>();
    if(!allowed.count(key) || !seen.insert(key).second) throw std::invalid_argument("geometry: unknown/duplicate field "+key);
  }
}
double positive(const YAML::Node& node) {
  const double value=node.as<double>();
  if(!std::isfinite(value) || value<=0) throw std::invalid_argument("geometry dimensions/ranges must be finite and positive");
  return value;
}
std::array<double,2> range(const YAML::Node& node) {
  if(!node.IsSequence() || node.size()!=2) throw std::invalid_argument("geometry ratio range requires [min,max]");
  std::array<double,2> result{positive(node[0]),positive(node[1])};
  if(result[0]>=result[1]) throw std::invalid_argument("geometry ratio min must be below max");
  return result;
}
}
GeometryResolver::GeometryResolver(const std::string& path) {
  const auto root=YAML::LoadFile(path);
  keys(root,{"small","large","small_ratio_range","large_ratio_range","label_policies"});
  keys(root["small"],{"width_m","height_m"}); keys(root["large"],{"width_m","height_m"});
  small_width_=positive(root["small"]["width_m"]); small_height_=positive(root["small"]["height_m"]);
  large_width_=positive(root["large"]["width_m"]); large_height_=positive(root["large"]["height_m"]);
  if(large_width_<=small_width_) throw std::invalid_argument("large width must exceed small width");
  small_ratio_=range(root["small_ratio_range"]); large_ratio_=range(root["large_ratio_range"]);
  const auto policies=root["label_policies"];
  if(!policies.IsMap() || policies.size()==0) throw std::invalid_argument("geometry label_policies must be a nonempty mapping");
  for(const auto& item:policies) {
    const int label=item.first.as<int>(); const auto policy=item.second.as<std::string>();
    if(label < -1 || label>7 || policies_.count(label)) throw std::invalid_argument("invalid/duplicate geometry label");
    if(policy!="auto" && policy!="small" && policy!="large" && policy!="model" && policy!="ignore")
      throw std::invalid_argument("geometry policy must be auto/small/large/model/ignore");
    policies_[label]=policy;
  }
}
GeometryChoice GeometryResolver::choose(const Candidate& candidate) const {
  GeometryChoice choice;
  const auto& p=candidate.corners;
  const double mean_length=(cv::norm(p[0]-p[3])+cv::norm(p[1]-p[2]))/2;
  if(!std::isfinite(mean_length) || mean_length<=0) { choice.status="invalid_geometry"; return choice; }
  choice.ratio=cv::norm((p[0]+p[3])*0.5F-(p[1]+p[2])*0.5F)/mean_length;
  if(!std::isfinite(choice.ratio)) { choice.ratio=0; choice.status="invalid_geometry"; return choice; }
  const auto found=policies_.find(static_cast<int>(candidate.label));
  if(found==policies_.end() || found->second=="ignore") { choice.status="label_filtered"; return choice; }
  const auto& policy=found->second;
  if(policy=="small" || policy=="large") {
    choice.size=policy=="small" ? PlateSize::small : PlateSize::large; choice.source="label_policy";
  } else if(candidate.size_hint!=PlateSize::unknown) {
    choice.size=candidate.size_hint; choice.source="model";
  } else if(policy=="model") { choice.status="model_size_unknown"; return choice; }
  else {
    choice.source="ratio";
    const bool small=choice.ratio>=small_ratio_[0] && choice.ratio<=small_ratio_[1];
    const bool large=choice.ratio>=large_ratio_[0] && choice.ratio<=large_ratio_[1];
    if(small && large) { choice.status="ambiguous_geometry"; return choice; }
    if(!small && !large) { choice.status="geometry_out_of_range"; return choice; }
    choice.size=small ? PlateSize::small : PlateSize::large;
  }
  choice.width_m=choice.size==PlateSize::small ? small_width_ : large_width_;
  choice.height_m=choice.size==PlateSize::small ? small_height_ : large_height_;
  choice.valid=true; choice.status="selected";
  return choice;
}
}  // namespace bof
