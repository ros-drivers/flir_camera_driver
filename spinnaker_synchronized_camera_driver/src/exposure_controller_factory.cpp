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

#include <spinnaker_synchronized_camera_driver/exposure_controller_factory.hpp>
#include <spinnaker_synchronized_camera_driver/follower_exposure_controller.hpp>
#include <spinnaker_synchronized_camera_driver/logging.hpp>
#include <spinnaker_synchronized_camera_driver/master_exposure_controller.hpp>

namespace spinnaker_synchronized_camera_driver
{
namespace exposure_controller_factory
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("cam_sync")); }
std::shared_ptr<spinnaker_camera_driver::ExposureController> newInstance(
  const std::string & type, const std::string & name, rclcpp::Node * node)
{
  if (type == "master") {
    return (std::make_shared<MasterExposureController>(name, node));
  } else if (type == "follower") {
    return (std::make_shared<FollowerExposureController>(name, node));
  }
  BOMB_OUT("unknown exposure controller type: " << type);
  return (nullptr);
}
}  // namespace exposure_controller_factory
}  // namespace spinnaker_synchronized_camera_driver
