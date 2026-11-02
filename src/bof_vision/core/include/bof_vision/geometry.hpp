#pragma once
#include "bof_vision/core.hpp"
#include <map>
#include <string>

namespace bof {
struct GeometryChoice {
  bool valid=false;
  PlateSize size=PlateSize::unknown;
  std::string source="none",status="geometry_unconfigured";
  double width_m=0,height_m=0,ratio=0;
};
// Explicit label policies + RPS-style light-centre distance / mean light length.
class GeometryResolver {
public:
  explicit GeometryResolver(const std::string& path);
  GeometryChoice choose(const Candidate& candidate) const;
private:
  double small_width_,small_height_,large_width_,large_height_;
  std::array<double,2> small_ratio_,large_ratio_;
  std::map<int,std::string> policies_;
};
}  // namespace bof
