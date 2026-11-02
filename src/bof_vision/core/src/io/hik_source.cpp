#include "bof_vision/camera.hpp"
#include <MvCameraControl.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace bof {
namespace {
void checked(int code,const char* operation) {
  if(code!=MV_OK) {
    char message[128];
    std::snprintf(message,sizeof(message),"%s failed (MVS 0x%08x)",operation,static_cast<unsigned>(code));
    throw std::runtime_error(message);
  }
}
struct Device {
  void* handle=nullptr;
  bool opened=false,grabbing=false;
  ~Device() {
    if(grabbing) MV_CC_StopGrabbing(handle);
    if(opened) MV_CC_CloseDevice(handle);
    if(handle) MV_CC_DestroyHandle(handle);
  }
};
struct FrameGuard {
  void* handle; MV_FRAME_OUT* frame;
  ~FrameGuard() { MV_CC_FreeImageBuffer(handle,frame); }
};
class HikSource final : public CameraSource {
public:
  HikSource(const std::string& serial,double exposure,double gain) {
    if(!std::isfinite(exposure) || exposure<=0) throw std::invalid_argument("invalid exposure_us");
    MV_CC_DEVICE_INFO_LIST list{};
    checked(MV_CC_EnumDevices(MV_USB_DEVICE,&list),"enumerate USB cameras");
    MV_CC_DEVICE_INFO* selected=nullptr;
    for(unsigned i=0;i<list.nDeviceNum;++i) {
      auto* raw=list.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber;
      const std::string found(reinterpret_cast<const char*>(raw));
      if((!serial.empty() && found==serial) || (serial.empty() && list.nDeviceNum==1)) {
        selected=list.pDeviceInfo[i]; serial_=found;
      }
    }
    if(!selected) throw std::runtime_error("no unique USB camera; connect camera or set serial_number");
    checked(MV_CC_CreateHandle(&device_.handle,selected),"create handle");
    checked(MV_CC_OpenDevice(device_.handle),"open camera"); device_.opened=true;
    checked(MV_CC_SetEnumValue(device_.handle,"TriggerMode",0),"disable trigger");
    set_exposure_us(exposure);
    if(gain>=0) set_gain(gain);
    checked(MV_CC_StartGrabbing(device_.handle),"start acquisition"); device_.grabbing=true;
  }
  std::string serial_number() const override { return serial_; }
  double set_exposure_us(double value) override {
    if(!std::isfinite(value) || value<=0) throw std::invalid_argument("invalid exposure_us");
    checked(MV_CC_SetEnumValue(device_.handle,"ExposureAuto",0),"disable automatic exposure");
    return set_float("ExposureTime",value);
  }
  double set_gain(double value) override {
    if(!std::isfinite(value) || value<0) throw std::invalid_argument("invalid gain");
    checked(MV_CC_SetEnumValue(device_.handle,"GainAuto",0),"disable automatic gain");
    return set_float("Gain",value);
  }
  std::optional<CameraFrame> read(unsigned timeout) override {
    MV_FRAME_OUT frame{};
    const int code=MV_CC_GetImageBuffer(device_.handle,&frame,timeout);
    if(static_cast<unsigned>(code)==MV_E_NODATA) return std::nullopt;
    checked(code,"get camera frame");
    FrameGuard guard{device_.handle,&frame};
    const auto received=std::chrono::steady_clock::now();
    cv::Mat bgr(frame.stFrameInfo.nHeight,frame.stFrameInfo.nWidth,CV_8UC3);
    MV_CC_PIXEL_CONVERT_PARAM convert{};
    convert.nWidth=frame.stFrameInfo.nWidth; convert.nHeight=frame.stFrameInfo.nHeight;
    convert.pSrcData=frame.pBufAddr; convert.nSrcDataLen=frame.stFrameInfo.nFrameLen;
    convert.enSrcPixelType=frame.stFrameInfo.enPixelType;
    convert.enDstPixelType=PixelType_Gvsp_BGR8_Packed;
    convert.pDstBuffer=bgr.data; convert.nDstBufferSize=static_cast<unsigned>(bgr.total()*bgr.elemSize());
    checked(MV_CC_ConvertPixelType(device_.handle,&convert),"convert to BGR8");
    return CameraFrame{std::move(bgr),frame.stFrameInfo.nFrameNum,received};
  }
private:
  double set_float(const char* key,double value) {
    MVCC_FLOATVALUE range{};
    checked(MV_CC_GetFloatValue(device_.handle,key,&range),"read camera parameter range");
    const auto applied=std::clamp(static_cast<float>(value),range.fMin,range.fMax);
    checked(MV_CC_SetFloatValue(device_.handle,key,applied),key);
    checked(MV_CC_GetFloatValue(device_.handle,key,&range),"read applied camera parameter");
    return range.fCurValue;
  }
  Device device_;
  std::string serial_;
};
}
std::unique_ptr<CameraSource> make_hik_camera(const std::string& serial,double exposure,double gain) {
  return std::make_unique<HikSource>(serial,exposure,gain);
}
}  // namespace bof
