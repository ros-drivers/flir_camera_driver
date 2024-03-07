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

#ifndef SPINNAKER_SYNCHRONIZED_CAMERA_DRIVER__EXPOSURE_CONTROLLER_FACTORY_HPP_
#define SPINNAKER_SYNCHRONIZED_CAMERA_DRIVER__EXPOSURE_CONTROLLER_FACTORY_HPP_

#include <memory>
#include <string>

namespace spinnaker_camera_driver
{
class ExposureController;  // forward decl
}
namespace rclcpp
{
class Node;  // forward decl
}
namespace spinnaker_synchronized_camera_driver
{
namespace exposure_controller_factory
{
std::shared_ptr<spinnaker_camera_driver::ExposureController> newInstance(
  const std::string & type, const std::string & name, rclcpp::Node * node);
}  // namespace exposure_controller_factory
}  // namespace spinnaker_synchronized_camera_driver
#endif  // SPINNAKER_SYNCHRONIZED_CAMERA_DRIVER__EXPOSURE_CONTROLLER_FACTORY_HPP_
