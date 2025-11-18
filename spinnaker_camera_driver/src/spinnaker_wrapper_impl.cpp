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
#include <spinnaker_camera_driver/logging.hpp>
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
    // nullptr means node was found, but is greyed out
    *msg = "node " + nodeName + " exists but is not accessible!";
    return (false);
  }
  if (!is_available(np)) {
    *msg = "node " + nodeName + " not available!";
    return (false);
  }
  if (!is_writable(np)) {
    *msg = "node " + nodeName + " not writable!";
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

SpinnakerWrapperImpl::SpinnakerWrapperImpl(rclcpp::Logger logger) : logger_(logger)
{
  system_ = Spinnaker::System::GetInstance();
  if (!system_) {
    LOG_ERROR("cannot instantiate spinnaker SDK!");
    throw std::runtime_error("failed to instantiate spinnaker SDK!");
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
  camera_ = nullptr;  // call destructor, may not be needed
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
  Lock lock(cameraMutex_);
  *retVal = "UNKNOWN";
  const auto np = genicam_utils::find_node(nodeName, camera_, debug_);
  if (!np) {
    return ("node " + nodeName + " not found!");
  }
  std::string msg;
  if (!common_checks(*np, nodeName, &msg)) {
    return (msg);
  }
  GenApi::CEnumerationPtr p = static_cast<GenApi::CEnumerationPtr>(*np);
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
      LOG_WARN("node " << nodeName << " invalid enum: " << val);
      LOG_WARN("allowed enum values: ");
      GenApi::StringList_t validValues;
      p->GetSymbolics(validValues);
      for (const auto & ve : validValues) {
        LOG_WARN("  " << ve);
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
  const auto np = genicam_utils::find_node(nodeName, cam, debug);
  if (!np) {
    return ("node " + nodeName + " not found!");
  }
  std::string msg;
  if (!common_checks(*np, nodeName, &msg)) {
    return (msg);
  }
  T1 p = static_cast<T1>(*np);
  p->SetValue(val);
  if (!is_readable(*np)) {
    return ("node " + nodeName + " current entry not readable!");
  }
  *retVal = p->GetValue();
  return ("OK");
}

std::string SpinnakerWrapperImpl::setDouble(const std::string & nn, double val, double * retVal)
{
  *retVal = std::nan("");
  Lock lock(cameraMutex_);
  return (set_parameter<GenApi::CFloatPtr, double>(nn, val, retVal, camera_, debug_));
}

std::string SpinnakerWrapperImpl::setBool(const std::string & nn, bool val, bool * retVal)
{
  Lock lock(cameraMutex_);
  *retVal = !val;
  return (set_parameter<GenApi::CBooleanPtr, bool>(nn, val, retVal, camera_, debug_));
}

std::string SpinnakerWrapperImpl::setInt(const std::string & nn, int val, int * retVal)
{
  Lock lock(cameraMutex_);
  *retVal = -1;
  return (set_parameter<GenApi::CIntegerPtr, int>(nn, val, retVal, camera_, debug_));
}

std::string SpinnakerWrapperImpl::execute(const std::string & nn)
{
  Lock lock(cameraMutex_);
  const auto np = genicam_utils::find_node(nn, camera_, debug_, true);
  if (!np) {
    return ("node " + nn + " not found!");
  }
  auto p = static_cast<GenApi::CCommandPtr>(*np);
  if (!is_available(p)) {
    return ("node " + nn + " not available!");
  }
  if (!is_writable(p)) {
    return ("node " + nn + " not writeable");
  }
  p->Execute();
  return ("OK");
}

static int int_ceil(size_t x, int y)
{
  // compute the integer ceil(x / y)
  return (static_cast<int>((x + static_cast<size_t>(y) - 1) / y));
}

static int16_t compute_brightness(
  pixel_format::PixelFormat pf, const uint8_t * data, size_t w, size_t h, size_t stride, int skip)
{
  if (!pixel_format::is_bayer(pf)) {
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

using StatusEnum = Spinnaker::GevIEEE1588StatusEnums;
static const std::map<StatusEnum, std::string> clockStatusMap = {
  {Spinnaker::GevIEEE1588Status_Initializing, "INI"},
  {Spinnaker::GevIEEE1588Status_Faulty, "FLT"},
  {Spinnaker::GevIEEE1588Status_Disabled, "DIS"},
  {Spinnaker::GevIEEE1588Status_Listening, "LIS"},
  {Spinnaker::GevIEEE1588Status_PreMaster, "PMAS"},
  {Spinnaker::GevIEEE1588Status_Master, "MAS"},
  {Spinnaker::GevIEEE1588Status_Passive, "PASV"},
  {Spinnaker::GevIEEE1588Status_Uncalibrated, "UNCL"},
  {Spinnaker::GevIEEE1588Status_Slave, "SLV"}};

std::string SpinnakerWrapperImpl::getIEEE1588Status() const
{
  const auto it = clockStatusMap.find(ptpStatus_);
  return (it == clockStatusMap.end() ? "INV" : it->second);
}

void SpinnakerWrapperImpl::getAndClearStatistics(SpinnakerWrapper::Stats * stats)
{
  // lock down the statistics
  Lock lock(statusMonitorMutex_);
  *stats = stats_;
  stats_.clear();
}

void SpinnakerWrapperImpl::OnImageEvent(Spinnaker::ImagePtr imgPtr)
{
  // make sure the status monitor is not modifying the camera
  // or other shared state like lastTime_
  Lock lock(statusMonitorMutex_);
  // update frame rate
  auto now = chrono::high_resolution_clock::now();
  const uint64_t t = chrono::duration_cast<chrono::nanoseconds>(now.time_since_epoch()).count();
  lastTime_ = t;
  stats_.numberReceived++;
  if (imgPtr->IsIncomplete()) {
    stats_.numberIncomplete++;
    numIncompleteImages_++;
  } else {
    float expTime = 0;
    float gain = 0;
    int64_t stamp = 0;

    try {
      const Spinnaker::ChunkData & chunk = imgPtr->GetChunkData();
      expTime = chunk.GetExposureTime();
      gain = chunk.GetGain();
      stamp = chunk.GetTimestamp();
    } catch (const Spinnaker::Exception & e) {
      // Without chunk data enabled there is no way to get e.g. the time stamps. Bad!
      // Spinnaker: Image does not contain chunk data. [-1001]
    }
    const uint32_t maxExpTime =
      static_cast<uint32_t>(is_readable(exposureTimeNode_) ? exposureTimeNode_->GetMax() : 0);
    if (useIEEE1588_) {
      Lock lock(cameraMutex_);
      ptpStatus_ = camera_->GevIEEE1588Status();
    }
#if 0
    std::cout << "got image: " << imgPtr->GetWidth() << "x" << imgPtr->GetHeight()
              << " stride: " << imgPtr->GetStride() << " ts: " << stamp << " exp time: " << expTime
              << " gain: " << gain << " bpp: " << imgPtr->GetBitsPerPixel()
              << " chan: " << imgPtr->GetNumChannels()
              << " tl payload type: " << imgPtr->GetTLPayloadType()
              << " tl pix fmt: " << imgPtr->GetTLPixelFormat()
              << " payload type: " << imgPtr->GetPayloadType()
              << " pixfmt: " << imgPtr->GetPixelFormat() << "(" << imgPtr->GetPixelFormatName()
              << ") int type: " << imgPtr->GetPixelFormatIntType()
              << " frame id: " << imgPtr->GetFrameID() << " img id: " << imgPtr->GetID()
              << " clock: " << getIEEE1588Status() << std::endl;
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
      imgPtr->GetFrameID(), pixelFormat_, numIncompleteImages_));
    const size_t fid = imgPtr->GetFrameID();
    if (lastFrameId_ != 0 && fid > lastFrameId_) {
      stats_.numberSkipped += fid - lastFrameId_ - 1;
    }
    lastFrameId_ = fid;
    numIncompleteImages_ = 0;
    callback_(img);
  }
}

bool SpinnakerWrapperImpl::initCamera(const std::string & serialNumber)
{
  Lock lock(cameraMutex_);
  if (camera_) {
    return false;
  }
  const auto interfaceList = system_->GetInterfaces();
  std::vector<std::string> failedInterfaces;

  for (size_t i = 0; i < interfaceList.GetSize(); i++) {
    const auto iface = interfaceList.GetByIndex(i);
    const auto & nodeMapInterface = iface->GetTLNodeMap();
    const auto ptrInterfaceType = nodeMapInterface.GetNode("InterfaceType");

    if (IsAvailable(ptrInterfaceType) && IsReadable(ptrInterfaceType)) {
      const Spinnaker::GenApi::CStringPtr ptrInterfaceDisplayName =
        nodeMapInterface.GetNode("InterfaceDisplayName");
      if (IsAvailable(ptrInterfaceDisplayName) && IsReadable(ptrInterfaceDisplayName)) {
        const auto interfaceDisplayName = ptrInterfaceDisplayName->GetValue();
        const auto camList = iface->GetCameras();
        for (size_t cam_idx = 0; cam_idx < camList.GetSize(); cam_idx++) {
          // try open the cameras in a specific interface
          Spinnaker::CameraPtr ptrCam = camList.GetByIndex(cam_idx);
          const auto serial = get_serial(ptrCam);
          if (serial == serialNumber) {
            try {
              ptrCam->Init();
              LOG_INFO_FMT(
                "Initialized camera [serial: %s] from: [%s]", serialNumber.c_str(),
                interfaceDisplayName.c_str());
              camera_ = ptrCam;
              return (true);
            } catch (Spinnaker::Exception & e) {
              failedInterfaces.push_back(interfaceDisplayName.c_str());
              // error while open the cameras in this interface
              ptrCam->DeInit();
            }
          }
        }
      } else {
        LOG_ERROR("Unknown Interface (Display name not readable)");
      }
    }
  }
  if (!camera_) {
    LOG_ERROR("Could not initialize camera on any interface!");
    for (const auto & iface : failedInterfaces) {
      LOG_ERROR("failed attempt on interface: " << iface);
    }
  }
  return (camera_ != 0);
}

bool SpinnakerWrapperImpl::deInitCamera()
{
  Lock lock(cameraMutex_);
  if (!camera_) {
    return (false);
  }
  camera_->DeInit();
  return (true);
}

bool SpinnakerWrapperImpl::startCamera(const SpinnakerWrapper::Callback & cb)
{
  Lock lock(cameraMutex_);
  if (!camera_ || cameraRunning_) {
    return false;
  }
  // switch on continuous acquisition
  // and get pixel format
  GenApi::INodeMap & nodeMap = camera_->GetNodeMap();
  if (set_acquisition_mode_continuous(nodeMap)) {
    camera_->RegisterEventHandler(*this);
    camera_->BeginAcquisition();
    {
      Lock lock(statusMonitorMutex_);
      keepRunning_ = true;
      thread_ = std::make_shared<std::thread>(&SpinnakerWrapperImpl::monitorStatus, this);
      cameraRunning_ = true;
    }

    GenApi::CEnumerationPtr ptrPixelFormat = nodeMap.GetNode("PixelFormat");
    if (GenApi::IsAvailable(ptrPixelFormat)) {
      setPixelFormat(ptrPixelFormat->GetCurrentEntry()->GetSymbolic().c_str());
    } else {
      setPixelFormat("BayerRG8");
      LOG_WARN("driver could not read pixel format!");
    }
    exposureTimeNode_ = nodeMap.GetNode("ExposureTime");
  } else {
    LOG_ERROR("failed to switch on continuous acquisition!");
    return (false);
  }
  callback_ = cb;
  return (true);
}

bool SpinnakerWrapperImpl::stopCamera()
{
  if (!cameraRunning_) {
    return (true);
  } else {
    LOG_INFO("stopping camera...");
  }
  Lock statusLock(statusMonitorMutex_);
  Lock cameraLock(cameraMutex_);
  int numRetries = 10;
  for (int i = 0; i < numRetries; i++) {
    if (camera_ && cameraRunning_) {
      if (thread_) {
        keepRunning_ = false;
        statusMonitorCv_.notify_all();
        statusLock.unlock();  // so the status thread can grab it
        thread_->join();
        thread_ = 0;
        statusLock.lock();  // status thread is done
      }
      try {
        // unregister the event handler first to avoid the case
        // where the driver is in OnImageEvent() when EndAcquisition() is called.
        try {
          camera_->UnregisterEventHandler(*this);
        } catch (const Spinnaker::Exception & e) {
          // ignore complaints that no events are registered
        }
        camera_->EndAcquisition();  // before unregistering the event handler!
        cameraRunning_ = false;
        LOG_INFO("camera stopped successfully.");
        return true;
      } catch (const std::exception & e) {
        LOG_ERROR(
          "failed to stop camera: " << e.what() << " will retry " << numRetries - 1 - i
                                    << " more times.");
      }
    }
  }
  LOG_WARN("cannot stop camera");
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
  Lock statusLock(statusMonitorMutex_);
  const int monitor_period_sec = 1;
  while (keepRunning_) {
    statusMonitorCv_.wait_until(
      statusLock, chrono::system_clock::now() + chrono::seconds(monitor_period_sec));
    const uint64_t t_now =
      chrono::duration_cast<chrono::nanoseconds>(chrono::system_clock::now().time_since_epoch())
        .count();
    if (t_now - lastTime_ > acquisitionTimeout_ && keepRunning_) {
      Lock lock(cameraMutex_);
      if (camera_) {
        LOG_WARN("Acquisition timeout, restarting streaming!");
        try {
          try {
            camera_->UnregisterEventHandler(*this);
          } catch (const Spinnaker::Exception & e) {
            // ignore complaints that no events are registered
          }
          try {
            camera_->EndAcquisition();
          } catch (const Spinnaker::Exception & e) {
            // ignore complaints that acquisition is not running
          }
          camera_->RegisterEventHandler(*this);
          camera_->BeginAcquisition();
        } catch (const Spinnaker::Exception & e) {
          LOG_WARN("restart attempt failed with error: " << e.what());
        }
      }
      stats_.acquisitionTimeouts++;
      stats_.acquisitionError = true;
    } else {
      stats_.acquisitionError = false;
    }
  }
  LOG_INFO("status monitoring thread exited.");
}

}  // namespace spinnaker_camera_driver
