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

#include <flir_spinnaker_ros2/camera_driver.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<flir_spinnaker_ros2::CameraDriver>(rclcpp::NodeOptions());

  RCLCPP_INFO(node->get_logger(), "camera_driver_node started up!");
  // actually run the node
  rclcpp::spin(node);  // should not return
  rclcpp::shutdown();
  return 0;
}
