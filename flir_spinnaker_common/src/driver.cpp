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

#include <Spinnaker.h>
#include <flir_spinnaker_common/driver.h>

#include <string>

#include "./driver_impl.h"

namespace flir_spinnaker_common
{
Driver::Driver() { driverImpl_.reset(new DriverImpl()); }

std::string Driver::getLibraryVersion() const
{
  return driverImpl_->getLibraryVersion();
}

void Driver::refreshCameraList() { driverImpl_->refreshCameraList(); }

std::vector<std::string> Driver::getSerialNumbers() const
{
  return driverImpl_->getSerialNumbers();
}

bool Driver::initCamera(const std::string & serialNumber)
{
  return driverImpl_->initCamera(serialNumber);
}

bool Driver::deInitCamera() { return driverImpl_->deInitCamera(); }

bool Driver::startCamera(const Callback & cb)
{
  return driverImpl_->startCamera(cb);
}

bool Driver::stopCamera() { return driverImpl_->stopCamera(); }

std::string Driver::getPixelFormat() const
{
  return driverImpl_->getPixelFormat();
}
double Driver::getReceiveFrameRate() const
{
  return (driverImpl_->getReceiveFrameRate());
}

std::string Driver::getNodeMapAsString()
{
  return (driverImpl_->getNodeMapAsString());
}

std::string Driver::setEnum(
  const std::string & nodeName, const std::string & val, std::string * retVal)
{
  try {
    return (driverImpl_->setEnum(nodeName, val, retVal));
  } catch (const Spinnaker::Exception & e) {
    throw DriverException(e.what());
  }
}

std::string Driver::setDouble(
  const std::string & nodeName, double val, double * retVal)
{
  try {
    return (driverImpl_->setDouble(nodeName, val, retVal));
  } catch (const Spinnaker::Exception & e) {
    throw DriverException(e.what());
  }
}

std::string Driver::setBool(
  const std::string & nodeName, bool val, bool * retVal)
{
  try {
    return (driverImpl_->setBool(nodeName, val, retVal));
  } catch (const Spinnaker::Exception & e) {
    throw DriverException(e.what());
  }
}

std::string Driver::setInt(const std::string & nodeName, int val, int * retVal)
{
  try {
    return (driverImpl_->setInt(nodeName, val, retVal));
  } catch (const Spinnaker::Exception & e) {
    throw DriverException(e.what());
  }
}

void Driver::setComputeBrightness(bool b)
{
  driverImpl_->setComputeBrightness(b);
}

void Driver::setAcquisitionTimeout(double t)
{
  driverImpl_->setAcquisitionTimeout(t);
}

void Driver::setDebug(bool b) { driverImpl_->setDebug(b); }

}  // namespace flir_spinnaker_common
