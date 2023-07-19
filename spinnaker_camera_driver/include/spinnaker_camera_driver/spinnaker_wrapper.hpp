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

#ifndef SPINNAKER_CAMERA_DRIVER__SPINNAKER_WRAPPER_HPP_
#define SPINNAKER_CAMERA_DRIVER__SPINNAKER_WRAPPER_HPP_

#include <functional>
#include <memory>
#include <spinnaker_camera_driver/image.hpp>
#include <string>
#include <vector>

namespace spinnaker_camera_driver
{
class SpinnakerWrapperImpl;
class SpinnakerWrapper
{
public:
  struct Exception : public std::exception
  {
    explicit Exception(const std::string & what) : what_(what) {}
    const char * what() const throw() { return what_.c_str(); }

  private:
    const std::string what_;
  };
  using Callback = std::function<void(const ImageConstPtr & img)>;

  SpinnakerWrapper();

  std::string getLibraryVersion() const;
  void refreshCameraList();
  std::vector<std::string> getSerialNumbers() const;

  bool initCamera(const std::string & serialNumber);
  bool deInitCamera();
  bool startCamera(const SpinnakerWrapper::Callback & cb);
  bool stopCamera();
  void setDebug(bool b);
  void setComputeBrightness(bool b);
  void setAcquisitionTimeout(double sec);

  std::string getPixelFormat() const;
  double getReceiveFrameRate() const;
  std::string getNodeMapAsString();
  std::string setEnum(const std::string & nodeName, const std::string & val, std::string * retVal);
  std::string setDouble(const std::string & nodeName, double val, double * retVal);
  std::string setBool(const std::string & nodeName, bool val, bool * retVal);
  std::string setInt(const std::string & nodeName, int val, int * retVal);

private:
  // ----- variables --
  std::shared_ptr<SpinnakerWrapperImpl> wrapperImpl_;
};
}  // namespace spinnaker_camera_driver
#endif  // SPINNAKER_CAMERA_DRIVER__SPINNAKER_WRAPPER_HPP_
