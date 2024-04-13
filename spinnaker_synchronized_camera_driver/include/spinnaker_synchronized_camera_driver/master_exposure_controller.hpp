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

#ifndef SPINNAKER_SYNCHRONIZED_CAMERA_DRIVER__MASTER_EXPOSURE_CONTROLLER_HPP_
#define SPINNAKER_SYNCHRONIZED_CAMERA_DRIVER__MASTER_EXPOSURE_CONTROLLER_HPP_

#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <spinnaker_camera_driver/exposure_controller.hpp>

namespace spinnaker_synchronized_camera_driver
{
class MasterExposureController : public spinnaker_camera_driver::ExposureController
{
public:
  using Camera = spinnaker_camera_driver::Camera;
  explicit MasterExposureController(const std::string & name, rclcpp::Node * n);
  void update(
    Camera * cam, const std::shared_ptr<const spinnaker_camera_driver::Image> & img) final;
  void addCamera(const std::shared_ptr<Camera> & cam) final;
  double getExposureTime() final { return (currentExposureTime_); };
  double getGain() final { return (currentGain_); }
  void link(const std::unordered_map<std::string, std::shared_ptr<ExposureController>> &) final {}

private:
  double calculateGain(double brightRatio) const;
  double calculateExposureTime(double brightRatio) const;
  bool changeExposure(double brightRatio, double minTime, double maxTime, const char * debugMsg);
  bool changeGain(double brightRatio, double minGain, double maxGain, const char * debugMsg);
  bool updateExposureWithGainPriority(double brightRatio);
  bool updateExposureWithTimePriority(double brightRatio);
  bool updateExposure(double b);
  rclcpp::Logger get_logger() { return (rclcpp::get_logger(cameraName_)); }

  template <class T>
  T declare_param(const std::string & n, const T & def)
  {
    return (node_->declare_parameter<T>(name_ + "." + n, def));
  }

  // ----------------- variables --------------------
  std::string name_;
  std::string cameraName_;
  rclcpp::Node * node_{0};
  int32_t targetBrightness_{128};
  std::string exposureParameterName_;
  std::string gainParameterName_;

  int brightnessTarget_{128};
  int brightnessTolerance_{5};
  double maxExposureTime_{1000};
  double minExposureTime_{0};
  double maxGain_{30};
  int currentBrightness_;
  double currentExposureTime_{0};
  double currentGain_{std::numeric_limits<float>::lowest()};
  int numFramesSkip_{0};
  int maxFramesSkip_{10};
  bool gainPriority_{false};
};
}  // namespace spinnaker_synchronized_camera_driver
#endif  // SPINNAKER_SYNCHRONIZED_CAMERA_DRIVER__MASTER_EXPOSURE_CONTROLLER_HPP_
