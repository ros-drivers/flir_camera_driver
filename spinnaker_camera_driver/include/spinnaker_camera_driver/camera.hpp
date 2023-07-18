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

#ifndef SPINNAKER_CAMERA_DRIVER__CAMERA_HPP_
#define SPINNAKER_CAMERA_DRIVER__CAMERA_HPP_

#include <string>

namespace spinnaker_camera_driver
{
class Camera
{
public:
  explicit Camera(const std::string & serial);

private:
  std::string void readParameters();
  // ----- variables --
  std::shared_ptr<rclcpp::Node> node_;
  image_transport::CameraPublisher pub_;
  std::shared_ptr<spinnaker_camera_driver::SpinnakerWrapper> wrapper_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager_;
};
}  // namespace spinnaker_camera_driver

#endif  // SPINNAKER_CAMERA_DRIVER__CAMERA_HPP_
