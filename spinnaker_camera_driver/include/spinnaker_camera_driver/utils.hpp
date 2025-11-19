// -*-c++-*--------------------------------------------------------------------
// Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef SPINNAKER_CAMERA_DRIVER__UTILS_HPP_
#define SPINNAKER_CAMERA_DRIVER__UTILS_HPP_

#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/rclcpp.hpp>

namespace spinnaker_camera_driver
{
namespace utils
{
#ifdef IMAGE_TRANSPORT_SUPPORTS_NODE_INTERFACES
std::shared_ptr<camera_info_manager::CameraInfoManager> makeCameraInfoManager(
  const std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> &,
  const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> &,
  const std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> &,
  const std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> &,
  const std::string & cameraName, const std::string & urlParameterName, uint32_t qSize);
#else
std::shared_ptr<camera_info_manager::CameraInfoManager> makeCameraInfoManager(
  rclcpp::Node * node, const std::string & cameraName, const std::string & parameterName);
#endif

}  // namespace utils
}  // namespace spinnaker_camera_driver
#endif  // SPINNAKER_CAMERA_DRIVER__UTILS_HPP_
