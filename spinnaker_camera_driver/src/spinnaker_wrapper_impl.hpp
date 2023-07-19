// -*-c++-*--------------------------------------------------------------------
// Copyright 2020 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef SPINNAKER_WRAPPER_IMPL_HPP_
#define SPINNAKER_WRAPPER_IMPL_HPP_

#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>

#include <memory>
#include <mutex>
#include <spinnaker_camera_driver/image.hpp>
#include <spinnaker_camera_driver/spinnaker_wrapper.hpp>
#include <string>
#include <thread>
#include <vector>

namespace spinnaker_camera_driver
{
class SpinnakerWrapperImpl : public Spinnaker::ImageEventHandler
{
public:
  SpinnakerWrapperImpl();
  ~SpinnakerWrapperImpl();
  // ------- inherited methods
  // from ImageEventHandler
  void OnImageEvent(Spinnaker::ImagePtr image) override;

  // ------- own methods
  std::string getLibraryVersion() const;
  void refreshCameraList();
  std::vector<std::string> getSerialNumbers() const;
  std::string getPixelFormat() const;

  bool initCamera(const std::string & serialNumber);
  bool deInitCamera();

  bool startCamera(const SpinnakerWrapper::Callback & cb);
  bool stopCamera();

  double getReceiveFrameRate() const;
  std::string getNodeMapAsString();
  // methods for setting camera params
  std::string setEnum(const std::string & nodeName, const std::string & val, std::string * retVal);
  std::string setDouble(const std::string & nodeName, double val, double * retVal);
  std::string setInt(const std::string & nodeName, int val, int * retVal);
  std::string setBool(const std::string & nodeName, bool val, bool * retVal);
  void setDebug(bool b) { debug_ = b; }
  void setComputeBrightness(bool b) { computeBrightness_ = b; }
  void setAcquisitionTimeout(double t) { acquisitionTimeout_ = static_cast<uint64_t>(t * 1e9); }

private:
  void setPixelFormat(const std::string & pixFmt);
  bool setInINodeMap(double f, const std::string & field, double * fret);
  void monitorStatus();

  // ----- variables --
  Spinnaker::SystemPtr system_;
  Spinnaker::CameraList cameraList_;
  Spinnaker::CameraPtr camera_;
  SpinnakerWrapper::Callback callback_;
  double avgTimeInterval_{0};
  uint64_t lastTime_{0};
  bool cameraRunning_{false};
  bool debug_{false};
  bool computeBrightness_{false};
  int brightnessSkipPixels_{32};
  pixel_format::PixelFormat pixelFormat_{pixel_format::INVALID};
  Spinnaker::GenApi::CFloatPtr exposureTimeNode_;
  bool keepRunning_{true};
  std::shared_ptr<std::thread> thread_;
  std::mutex mutex_;
  uint64_t acquisitionTimeout_{10000000000ULL};
};
}  // namespace spinnaker_camera_driver

#endif  // SPINNAKER_WRAPPER_IMPL_HPP_
