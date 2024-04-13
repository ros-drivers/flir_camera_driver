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

#include <rclcpp/rclcpp.hpp>
#include <spinnaker_camera_driver/camera.hpp>
#include <spinnaker_synchronized_camera_driver/follower_exposure_controller.hpp>
#include <spinnaker_synchronized_camera_driver/logging.hpp>

namespace spinnaker_synchronized_camera_driver
{
FollowerExposureController::FollowerExposureController(
  const std::string & name, rclcpp::Node * node)
: name_(name), node_(node)
{
  exposureParameterName_ = declare_param<std::string>("exposure_parameter", "exposure_time");
  gainParameterName_ = declare_param<std::string>("gain_parameter", "gain");
  maxFramesSkip_ = declare_param<int>("max_frames_skip", 10);  // number of frames to wait
  masterControllerName_ = declare_param<std::string>("master", "");
  if (masterControllerName_.empty()) {
    BOMB_OUT("master exposure controller must be set for controller " << name_);
  }
}
void FollowerExposureController::link(
  const std::unordered_map<std::string, std::shared_ptr<ExposureController>> & map)
{
  const auto it = map.find(masterControllerName_);
  if (it == map.end()) {
    BOMB_OUT("cannot find master " << masterControllerName_ << " for controller " << name_);
  }
  masterController_ = it->second;
}

void FollowerExposureController::update(
  spinnaker_camera_driver::Camera * cam,
  const std::shared_ptr<const spinnaker_camera_driver::Image> & img)
{
  // if the exposure parameters are not set yet, set them now
  if (currentExposureTime_ == 0) {
    currentExposureTime_ = static_cast<double>(img->exposureTime_);
  }
  if (currentGain_ == std::numeric_limits<float>::lowest()) {
    currentGain_ = img->gain_;
  }
  // check if the exposure and brightness settings reported along with the image
  // match what last has been sent to the camera.
  if (
    fabs(currentGain_ - img->gain_) <= 0.05 * (currentGain_ + img->gain_) &&
    fabs(currentExposureTime_ - static_cast<double>(img->exposureTime_)) <=
      0.05 * (currentExposureTime_ + static_cast<double>(img->exposureTime_)) &&
    numFramesSkip_ < maxFramesSkip_) {
    numFramesSkip_ = 0;  // no skipping anymore!
  }

  if (numFramesSkip_ > 0) {
    // Changes in gain or shutter take a few
    // frames to arrive at the camera, so we skip those.
    numFramesSkip_--;
  } else {
    const auto masterExposureTime = masterController_->getExposureTime();
    const auto masterGain = masterController_->getGain();
    bool parametersChanged{false};
    if (masterExposureTime != currentExposureTime_) {
      const auto expName = cam->getPrefix() + exposureParameterName_;
      node_->set_parameter(rclcpp::Parameter(expName, masterExposureTime));
      parametersChanged = true;
    }
    if (masterGain != currentGain_) {
      const auto gainName = cam->getPrefix() + gainParameterName_;
      node_->set_parameter(rclcpp::Parameter(gainName, masterGain));
      parametersChanged = true;
    }
    if (parametersChanged) {
      const int b = std::min(std::max(1, static_cast<int>(img->brightness_)), 255);
      LOG_INFO(
        "bright " << b << " at time/gain: [" << currentExposureTime_ << " " << currentGain_
                  << "] new: [" << masterExposureTime << " " << masterGain << "]");
      numFramesSkip_ = maxFramesSkip_;  // restart frame skipping
      currentExposureTime_ = masterExposureTime;
      currentGain_ = masterGain;
    }
  }
}

void FollowerExposureController::addCamera(
  const std::shared_ptr<spinnaker_camera_driver::Camera> & cam)
{
  cameraName_ = cam->getName();
}

}  // namespace spinnaker_synchronized_camera_driver
