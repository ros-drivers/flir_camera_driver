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

#ifndef SPINNAKER_SYNCHRONIZED_CAMERA_DRIVER__SYNCHRONIZED_CAMERA_DRIVER_HPP_
#define SPINNAKER_SYNCHRONIZED_CAMERA_DRIVER__SYNCHRONIZED_CAMERA_DRIVER_HPP_

#include <image_transport/image_transport.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <spinnaker_camera_driver/camera.hpp>
#include <spinnaker_synchronized_camera_driver/time_keeper.hpp>
#include <thread>
#include <unordered_map>

namespace spinnaker_camera_driver
{
class ExposureController;
}

namespace spinnaker_synchronized_camera_driver
{
class TimeEstimator;
class SynchronizedCameraDriver : public rclcpp::Node
{
public:
  explicit SynchronizedCameraDriver(const rclcpp::NodeOptions & options);
  ~SynchronizedCameraDriver();
  bool update(size_t idx, uint64_t hostTime, double dt, uint64_t * frameTime);

private:
  void createCameras();
  void createExposureControllers();
  void printStatus();
  // ----- variables --
  std::shared_ptr<image_transport::ImageTransport> imageTransport_;
  std::map<const std::string, std::shared_ptr<spinnaker_camera_driver::Camera>> cameras_;
  std::vector<std::shared_ptr<TimeKeeper>> timeKeepers_;
  rclcpp::TimerBase::SharedPtr statusTimer_;
  double avgFrameInterval_{-1};
  std::mutex mutex_;
  size_t numUpdatesRequired_{0};
  size_t numUpdatesReceived_{0};
  std::shared_ptr<TimeEstimator> timeEstimator_;
  std::unordered_map<std::string, std::shared_ptr<spinnaker_camera_driver::ExposureController>>
    exposureControllers_;
};
}  // namespace spinnaker_synchronized_camera_driver
#endif  // SPINNAKER_SYNCHRONIZED_CAMERA_DRIVER__SYNCHRONIZED_CAMERA_DRIVER_HPP_
