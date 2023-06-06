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

#ifndef FLIR_SPINNAKER_COMMON__DRIVER_H_
#define FLIR_SPINNAKER_COMMON__DRIVER_H_

#include <flir_spinnaker_common/image.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace flir_spinnaker_common
{
class DriverImpl;
class Driver
{
public:
  struct DriverException : public std::exception
  {
    explicit DriverException(const std::string & what) : what_(what) {}
    const char * what() const throw() { return what_.c_str(); }

  private:
    const std::string what_;
  };
  typedef std::function<void(const ImageConstPtr & img)> Callback;
  Driver();
  std::string getLibraryVersion() const;
  void refreshCameraList();
  std::vector<std::string> getSerialNumbers() const;

  bool initCamera(const std::string & serialNumber);
  bool deInitCamera();
  bool startCamera(const Driver::Callback & cb);
  bool stopCamera();
  void setDebug(bool b);
  void setComputeBrightness(bool b);
  void setAcquisitionTimeout(double sec);

  std::string getPixelFormat() const;
  double getReceiveFrameRate() const;
  std::string getNodeMapAsString();
  std::string setEnum(
    const std::string & nodeName, const std::string & val,
    std::string * retVal);
  std::string setDouble(
    const std::string & nodeName, double val, double * retVal);
  std::string setBool(const std::string & nodeName, bool val, bool * retVal);
  std::string setInt(const std::string & nodeName, int val, int * retVal);

private:
  // ----- variables --
  std::shared_ptr<DriverImpl> driverImpl_;
};
}  // namespace flir_spinnaker_common
#endif  // FLIR_SPINNAKER_COMMON__DRIVER_H_
