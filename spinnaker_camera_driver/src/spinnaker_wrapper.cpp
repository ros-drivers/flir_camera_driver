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

#include <Spinnaker.h>

#include <spinnaker_camera_driver/spinnaker_wrapper.hpp>
#include <string>

#include "./spinnaker_wrapper_impl.hpp"

namespace spinnaker_camera_driver
{
SpinnakerWrapper::SpinnakerWrapper() { wrapperImpl_.reset(new SpinnakerWrapperImpl()); }

std::string SpinnakerWrapper::getLibraryVersion() const
{
  return wrapperImpl_->getLibraryVersion();
}

void SpinnakerWrapper::refreshCameraList() { wrapperImpl_->refreshCameraList(); }

std::vector<std::string> SpinnakerWrapper::getSerialNumbers() const
{
  return wrapperImpl_->getSerialNumbers();
}

bool SpinnakerWrapper::initCamera(const std::string & serialNumber)
{
  return wrapperImpl_->initCamera(serialNumber);
}

bool SpinnakerWrapper::deInitCamera() { return wrapperImpl_->deInitCamera(); }

bool SpinnakerWrapper::startCamera(const Callback & cb) { return wrapperImpl_->startCamera(cb); }

bool SpinnakerWrapper::stopCamera() { return wrapperImpl_->stopCamera(); }

std::string SpinnakerWrapper::getPixelFormat() const { return wrapperImpl_->getPixelFormat(); }
double SpinnakerWrapper::getReceiveFrameRate() const
{
  return (wrapperImpl_->getReceiveFrameRate());
}

std::string SpinnakerWrapper::getNodeMapAsString() { return (wrapperImpl_->getNodeMapAsString()); }

std::string SpinnakerWrapper::setEnum(
  const std::string & nodeName, const std::string & val, std::string * retVal)
{
  try {
    return (wrapperImpl_->setEnum(nodeName, val, retVal));
  } catch (const Spinnaker::Exception & e) {
    throw SpinnakerWrapper::Exception(e.what());
  }
}

std::string SpinnakerWrapper::setDouble(const std::string & nodeName, double val, double * retVal)
{
  try {
    return (wrapperImpl_->setDouble(nodeName, val, retVal));
  } catch (const Spinnaker::Exception & e) {
    throw SpinnakerWrapper::Exception(e.what());
  }
}

std::string SpinnakerWrapper::setBool(const std::string & nodeName, bool val, bool * retVal)
{
  try {
    return (wrapperImpl_->setBool(nodeName, val, retVal));
  } catch (const Spinnaker::Exception & e) {
    throw SpinnakerWrapper::Exception(e.what());
  }
}

std::string SpinnakerWrapper::setInt(const std::string & nodeName, int val, int * retVal)
{
  try {
    return (wrapperImpl_->setInt(nodeName, val, retVal));
  } catch (const Spinnaker::Exception & e) {
    throw SpinnakerWrapper::Exception(e.what());
  }
}

void SpinnakerWrapper::setComputeBrightness(bool b) { wrapperImpl_->setComputeBrightness(b); }

void SpinnakerWrapper::setAcquisitionTimeout(double t) { wrapperImpl_->setAcquisitionTimeout(t); }

void SpinnakerWrapper::setDebug(bool b) { wrapperImpl_->setDebug(b); }

}  // namespace spinnaker_camera_driver
