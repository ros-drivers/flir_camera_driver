// -*-c++-*--------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "spinnaker_wrapper_impl.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "genicam_utils.hpp"

namespace spinnaker_camera_driver
{
namespace chrono = std::chrono;
namespace GenApi = Spinnaker::GenApi;
namespace GenICam = Spinnaker::GenICam;

template <class T>
static bool is_available(T ptr)
{
  return (ptr.IsValid() && GenApi::IsAvailable(ptr));
}

template <class T>
static bool is_writable(T ptr)
{
  return (ptr.IsValid() && GenApi::IsAvailable(ptr) && GenApi::IsWritable(ptr));
}

template <class T>
static bool is_readable(T ptr)
{
  return (ptr.IsValid() && GenApi::IsAvailable(ptr) && GenApi::IsReadable(ptr));
}

static bool common_checks(
  const GenApi::CNodePtr & np, const std::string & nodeName, std::string * msg)
{
  if (!np.IsValid()) {
    *msg = "node " + nodeName + " does not exist!";
    return (false);
  }
  if (!is_available(np)) {
    *msg = "node " + nodeName + " not available!";
    return (false);
  }
  if (!is_writable(np)) {
    *msg = "node " + nodeName + " not available!";
    return (false);
  }
  return (true);
}

static std::string get_serial(Spinnaker::CameraPtr cam)
{
  const auto & nodeMap = cam->GetTLDeviceNodeMap();
  const Spinnaker::GenApi::CStringPtr psn = nodeMap.GetNode("DeviceSerialNumber");
  return is_readable(psn) ? std::string(psn->GetValue()) : "";
}

static bool set_acquisition_mode_continuous(GenApi::INodeMap & nodeMap)
{
  Spinnaker::GenApi::CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
  if (GenApi::IsAvailable(ptrAcquisitionMode) && GenApi::IsWritable(ptrAcquisitionMode)) {
    GenApi::CEnumEntryPtr ptrAcquisitionModeContinuous =
      ptrAcquisitionMode->GetEntryByName("Continuous");
    if (
      GenApi::IsAvailable(ptrAcquisitionModeContinuous) &&
      GenApi::IsReadable(ptrAcquisitionModeContinuous)) {
      // Retrieve integer value from entry node
      const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
      // Set integer value from entry node as new value of enumeration node
      ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
      return true;
    }
  }
  return false;
}

SpinnakerWrapperImpl::SpinnakerWrapperImpl()
{
  system_ = Spinnaker::System::GetInstance();
  if (!system_) {
    std::cerr << "cannot instantiate spinnaker driver!" << std::endl;
    throw std::runtime_error("failed to get spinnaker driver!");
  }
  refreshCameraList();
}

void SpinnakerWrapperImpl::refreshCameraList()
{
  cameraList_ = system_->GetCameras();
  for (size_t cam_idx = 0; cam_idx < cameraList_.GetSize(); cam_idx++) {
    const auto cam = cameraList_[cam_idx];
  }
}

SpinnakerWrapperImpl::~SpinnakerWrapperImpl()
{
  keepRunning_ = false;
  stopCamera();
  deInitCamera();
  camera_ = 0;  // call destructor, may not be needed
  cameraList_.Clear();
  if (system_) {
    system_->ReleaseInstance();
  }
}

std::string SpinnakerWrapperImpl::getLibraryVersion() const
{
  const Spinnaker::LibraryVersion lv = system_->GetLibraryVersion();
  char buf[256];
  snprintf(buf, sizeof(buf), "%d.%d.%d.%d", lv.major, lv.minor, lv.type, lv.build);
  return std::string(buf);
}

std::vector<std::string> SpinnakerWrapperImpl::getSerialNumbers() const
{
  std::vector<std::string> sn;
  for (size_t cam_idx = 0; cam_idx < cameraList_.GetSize(); cam_idx++) {
    const auto cam = cameraList_.GetByIndex(cam_idx);
    sn.push_back(get_serial(cam));
  }
  return sn;
}

std::string SpinnakerWrapperImpl::setEnum(
  const std::string & nodeName, const std::string & val, std::string * retVal)
{
  *retVal = "UNKNOWN";
  GenApi::CNodePtr np = genicam_utils::find_node(nodeName, camera_, debug_);
  std::string msg;
  if (!common_checks(np, nodeName, &msg)) {
    return (msg);
  }
  GenApi::CEnumerationPtr p = static_cast<GenApi::CEnumerationPtr>(np);
  if (!is_writable(p)) {
    return ("node " + nodeName + " not enum???");
  }
  // find integer corresponding to the enum string
  GenApi::CEnumEntryPtr setVal = p->GetEntryByName(val.c_str());
  if (!is_readable(setVal)) {
    // bad enum value, try to read current value nevertheless
    if (is_readable(p)) {
      auto ce = p->GetCurrentEntry();
      if (ce) {
        *retVal = ce->GetSymbolic().c_str();
      }
    }
    if (debug_) {
      std::cout << "node " << nodeName << " invalid enum: " << val << std::endl;
      std::cout << "allowed enum values: " << std::endl;
      GenApi::StringList_t validValues;
      p->GetSymbolics(validValues);
      for (const auto & ve : validValues) {
        std::cout << "  " << ve << std::endl;
      }
    }
    return ("node " + nodeName + " invalid enum: " + val);
  }
  // set the new enum value
  p->SetIntValue(setVal->GetValue());
  // read it back
  if (is_readable(p)) {
    auto ce = p->GetCurrentEntry();
    if (ce) {
      *retVal = ce->GetSymbolic().c_str();
    } else {
      return ("node " + nodeName + " current entry not readable!");
    }
  } else {
    return ("node " + nodeName + " is not readable!");
  }
  return ("OK");
}

template <class T>
T set_invalid()
{
  return (std::nan(""));
}

template <>
int set_invalid()
{
  return (-1);
}

template <class T1, class T2>
static std::string set_parameter(
  const std::string & nodeName, T2 val, T2 * retVal, const Spinnaker::CameraPtr & cam, bool debug)
{
  *retVal = set_invalid<T2>();
  GenApi::CNodePtr np = genicam_utils::find_node(nodeName, cam, debug);
  std::string msg;
  if (!common_checks(np, nodeName, &msg)) {
    return (msg);
  }
  T1 p = static_cast<T1>(np);
  p->SetValue(val);
  if (!is_readable(np)) {
    return ("node " + nodeName + " current entry not readable!");
  }
  *retVal = p->GetValue();
  return ("OK");
}

std::string SpinnakerWrapperImpl::setDouble(const std::string & nn, double val, double * retVal)
{
  *retVal = std::nan("");
  return (set_parameter<GenApi::CFloatPtr, double>(nn, val, retVal, camera_, debug_));
}

std::string SpinnakerWrapperImpl::setBool(const std::string & nn, bool val, bool * retVal)
{
  *retVal = !val;
  return (set_parameter<GenApi::CBooleanPtr, bool>(nn, val, retVal, camera_, debug_));
}

std::string SpinnakerWrapperImpl::setInt(const std::string & nn, int val, int * retVal)
{
  *retVal = -1;
  return (set_parameter<GenApi::CIntegerPtr, int>(nn, val, retVal, camera_, debug_));
}

double SpinnakerWrapperImpl::getReceiveFrameRate() const
{
  return (avgTimeInterval_ > 0 ? (1.0 / avgTimeInterval_) : 0);
}

static int int_ceil(size_t x, int y)
{
  // compute the integer ceil(x / y)
  return (static_cast<int>((x + static_cast<size_t>(y) - 1) / y));
}

static int16_t compute_brightness(
  pixel_format::PixelFormat pf, const uint8_t * data, size_t w, size_t h, size_t stride, int skip)
{
  if (pf != pixel_format::BayerRG8) {
    return (0);
  }
  const uint64_t cnt = int_ceil(w, skip) * int_ceil(h, skip);
  uint64_t tot = 0;
  const uint8_t * p = data;
  for (size_t row = 0; row < h; row += skip) {
    for (size_t col = 0; col < w; col += skip) {
      tot += p[col];
    }
    p += stride * skip;
  }
  return (tot / cnt);
}

void SpinnakerWrapperImpl::OnImageEvent(Spinnaker::ImagePtr imgPtr)
{
  // update frame rate
  auto now = chrono::high_resolution_clock::now();
  uint64_t t = chrono::duration_cast<chrono::nanoseconds>(now.time_since_epoch()).count();
  if (avgTimeInterval_ == 0) {
    if (lastTime_ != 0) {
      avgTimeInterval_ = (t - lastTime_) * 1e-9;
    }
  } else {
    const double dt = (t - lastTime_) * 1e-9;
    const double alpha = 0.01;
    avgTimeInterval_ = avgTimeInterval_ * (1.0 - alpha) + dt * alpha;
  }
  {
    std::unique_lock<std::mutex> lock(mutex_);
    lastTime_ = t;
  }

  if (imgPtr->IsIncomplete()) {
    // Retrieve and print the image status description
    std::cout << "Image incomplete: "
              << Spinnaker::Image::GetImageStatusDescription(imgPtr->GetImageStatus()) << std::endl;
  } else {
    const Spinnaker::ChunkData & chunk = imgPtr->GetChunkData();
    const float expTime = chunk.GetExposureTime();
    const float gain = chunk.GetGain();
    const int64_t stamp = chunk.GetTimestamp();
    const uint32_t maxExpTime =
      static_cast<uint32_t>(is_readable(exposureTimeNode_) ? exposureTimeNode_->GetMax() : 0);

#if 0
    std::cout << "got image: " << imgPtr->GetWidth() << "x"
              << imgPtr->GetHeight() << " stride: " << imgPtr->GetStride()
              << " ts: " << stamp << " exp time: " << expTime
              << " gain: " << gain << " bpp: " << imgPtr->GetBitsPerPixel()
              << " chan: " << imgPtr->GetNumChannels()
              << " tl payload type: " << imgPtr->GetTLPayloadType()
              << " tl pix fmt: " << imgPtr->GetTLPixelFormat()
              << " payload type: " << imgPtr->GetPayloadType()
              << " pixfmt enum: " << imgPtr->GetPixelFormat()
              << " fmt: " << imgPtr->GetPixelFormatName()
              << " int type: " << imgPtr->GetPixelFormatIntType()
              << " frame id: " << imgPtr->GetFrameID()
              << " img id: " << imgPtr->GetID() << std::endl;
#endif
    // Note: GetPixelFormat() did not work for the grasshopper, so ignoring
    // pixel format in image, using the one from the configuration
    const int16_t brightness =
      computeBrightness_
        ? compute_brightness(
            pixelFormat_, static_cast<const uint8_t *>(imgPtr->GetData()), imgPtr->GetWidth(),
            imgPtr->GetHeight(), imgPtr->GetStride(), brightnessSkipPixels_)
        : -1;
    ImagePtr img(new Image(
      t, brightness, expTime, maxExpTime, gain, stamp, imgPtr->GetImageSize(),
      imgPtr->GetImageStatus(), imgPtr->GetData(), imgPtr->GetWidth(), imgPtr->GetHeight(),
      imgPtr->GetStride(), imgPtr->GetBitsPerPixel(), imgPtr->GetNumChannels(),
      imgPtr->GetFrameID(), pixelFormat_));
    callback_(img);
  }
}

bool SpinnakerWrapperImpl::initCamera(const std::string & serialNumber)
{
  if (camera_) {
    return false;
  }
  for (size_t cam_idx = 0; cam_idx < cameraList_.GetSize(); cam_idx++) {
    auto cam = cameraList_.GetByIndex(cam_idx);
    const std::string sn = get_serial(cam);
    if (sn == serialNumber) {
      camera_ = cam;
      camera_->Init();
      break;
    }
  }
  return (camera_ != 0);
}

bool SpinnakerWrapperImpl::deInitCamera()
{
  if (!camera_) {
    return (false);
  }
  camera_->DeInit();
  return (true);
}

bool SpinnakerWrapperImpl::startCamera(const SpinnakerWrapper::Callback & cb)
{
  if (!camera_ || cameraRunning_) {
    return false;
  }
  // switch on continuous acquisition
  // and get pixel format
  GenApi::INodeMap & nodeMap = camera_->GetNodeMap();
  if (set_acquisition_mode_continuous(nodeMap)) {
    camera_->RegisterEventHandler(*this);
    camera_->BeginAcquisition();
    thread_ = std::make_shared<std::thread>(&SpinnakerWrapperImpl::monitorStatus, this);
    cameraRunning_ = true;

    GenApi::CEnumerationPtr ptrPixelFormat = nodeMap.GetNode("PixelFormat");
    if (GenApi::IsAvailable(ptrPixelFormat)) {
      setPixelFormat(ptrPixelFormat->GetCurrentEntry()->GetSymbolic().c_str());
    } else {
      setPixelFormat("BayerRG8");
      std::cerr << "WARNING: driver could not read pixel format!" << std::endl;
    }
    exposureTimeNode_ = nodeMap.GetNode("ExposureTime");
  } else {
    std::cerr << "failed to switch on continuous acquisition!" << std::endl;
    return (false);
  }
  callback_ = cb;
  return (true);
}

bool SpinnakerWrapperImpl::stopCamera()
{
  if (camera_ && cameraRunning_) {
    if (thread_) {
      keepRunning_ = false;
      thread_->join();
      thread_ = 0;
    }
    camera_->EndAcquisition();  // before unregistering the event handler!
    camera_->UnregisterEventHandler(*this);

    cameraRunning_ = false;
    return true;
  }
  return (false);
}

void SpinnakerWrapperImpl::setPixelFormat(const std::string & pixFmt)
{
  pixelFormat_ = pixel_format::from_nodemap_string(pixFmt);
}

std::string SpinnakerWrapperImpl::getPixelFormat() const
{
  return (pixel_format::to_string(pixelFormat_));
}

std::string SpinnakerWrapperImpl::getNodeMapAsString()
{
  std::stringstream ss;
  genicam_utils::get_nodemap_as_string(ss, camera_);
  return (ss.str());
}

void SpinnakerWrapperImpl::monitorStatus()
{
  while (keepRunning_) {
    std::this_thread::sleep_for(chrono::seconds(1));
    uint64_t lastTime;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      lastTime = lastTime_;
    }
    auto now = chrono::high_resolution_clock::now();
    uint64_t t = chrono::duration_cast<chrono::nanoseconds>(now.time_since_epoch()).count();
    if (t - lastTime > acquisitionTimeout_ && camera_) {
      std::cout << "WARNING: acquisition timeout, restarting!" << std::endl;
      // Mucking with the camera in this thread without proper
      // locking does not feel good. Expect some rare crashes.
      camera_->EndAcquisition();
      camera_->BeginAcquisition();
    }
  }
}

}  // namespace spinnaker_camera_driver
