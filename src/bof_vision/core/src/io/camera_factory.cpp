#include "bof_vision/camera.hpp"
#include <stdexcept>
namespace bof {
bool hik_camera_enabled() {
#ifdef BOF_WITH_MVS
  return true;
#else
  return false;
#endif
}
}
#ifndef BOF_WITH_MVS
namespace bof {
std::unique_ptr<CameraSource> make_hik_camera(const std::string&,double,double) {
  throw std::runtime_error("camera source needs a build with BOF_WITH_MVS=ON");
}
}
#endif
