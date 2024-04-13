// -*-c++-*--------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef SPINNAKER_CAMERA_DRIVER__EXPOSURE_CONTROLLER_HPP_
#define SPINNAKER_CAMERA_DRIVER__EXPOSURE_CONTROLLER_HPP_

#include <memory>

namespace spinnaker_camera_driver
{
class Image;
class Camera;

class ExposureController
{
public:
  ExposureController() = default;
  virtual ~ExposureController() {}
  virtual void update(Camera * cam, const std::shared_ptr<const Image> & img) = 0;
  virtual void addCamera(const std::shared_ptr<Camera> & cam) = 0;
  virtual double getExposureTime() = 0;
  virtual double getGain() = 0;
  virtual void link(
    const std::unordered_map<std::string, std::shared_ptr<ExposureController>> & map) = 0;
};
}  // namespace spinnaker_camera_driver
#endif  // SPINNAKER_CAMERA_DRIVER__EXPOSURE_CONTROLLER_HPP_
