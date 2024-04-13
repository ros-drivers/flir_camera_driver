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
#include <spinnaker_synchronized_camera_driver/logging.hpp>
#include <spinnaker_synchronized_camera_driver/master_exposure_controller.hpp>

// #define DEBUG

namespace spinnaker_synchronized_camera_driver
{
MasterExposureController::MasterExposureController(const std::string & name, rclcpp::Node * node)
: name_(name), node_(node)
{
  exposureParameterName_ = declare_param<std::string>("exposure_parameter", "exposure_time");
  gainParameterName_ = declare_param<std::string>("gain_parameter", "gain");
  brightnessTarget_ = std::min(std::max(declare_param<int>("brightness_target", 120), 1), 255);
  currentBrightness_ = brightnessTarget_;
  brightnessTolerance_ = declare_param<int>("brightness_tolerance", 5);
  maxExposureTime_ = std::max(declare_param<int>("max_exposure_time", 1000), 1);
  minExposureTime_ = std::max(declare_param<int>("min_exposure_time", 10), 1);
  maxGain_ = declare_param<double>("max_gain", 10);
  gainPriority_ = declare_param<bool>("gain_priority", false);
  maxFramesSkip_ = declare_param<int>("max_frames_skip", 10);  // number of frames to wait
}

double MasterExposureController::calculateGain(double brightRatio) const
{
  // because gain is in db:
  // db(G) = 10 * log_10(G) = 10 * ln(G) / ln(10) = 4.34 * ln(G)
  const double kp = 4.34;
  const double desiredGain = currentGain_ + kp * log(brightRatio);
  const double cappedGain = std::max(std::min(desiredGain, maxGain_), 0.0);
  // below threshold set to zero because it can no longer be set accurately
  // at the camera
  return (cappedGain > 0.5 ? cappedGain : 0);
}

double MasterExposureController::calculateExposureTime(double brightRatio) const
{
  const double desiredExposureTime = currentExposureTime_ * brightRatio;
  const double optTime = std::max(0.0, std::min(desiredExposureTime, maxExposureTime_));
  return (optTime);
}

bool MasterExposureController::changeExposure(
  double brightRatio, double minTime, double maxTime, const char * debugMsg)
{
  const double optTime = std::min(std::max(calculateExposureTime(brightRatio), minTime), maxTime);
  if (currentExposureTime_ != optTime) {
    currentExposureTime_ = optTime;
#ifdef DEBUG
    LOG_INFO(debugMsg);
#else
    (void)debugMsg;
#endif
    return (true);
  }
  return (false);
}

bool MasterExposureController::changeGain(
  double brightRatio, double minGain, double maxGain, const char * debugMsg)
{
  const double optGain = std::min(std::max(calculateGain(brightRatio), minGain), maxGain);
  if (optGain != currentGain_) {
    currentGain_ = optGain;
#ifdef DEBUG
    LOG_INFO(debugMsg);
#else
    (void)debugMsg;
#endif
    return (true);
  }
  return (false);
}

bool MasterExposureController::updateExposureWithGainPriority(double brightRatio)
{
  if (brightRatio < 1) {  // image is too bright
    if (currentGain_ > 0) {
      // if gain is nonzero, dial it back before touching exposure times
      if (changeGain(brightRatio, 0, maxGain_, "gp: --gain!")) {
        return (true);
      }
    } else {
      // gain is already at zero, reduce exposure time
      if (changeExposure(brightRatio, 0, maxExposureTime_, "gp: --time!")) {
        return (true);
      }
    }
  } else {  // image is too dark
    if (currentExposureTime_ < maxExposureTime_) {
      // first try bumping the exposure time
      if (changeExposure(brightRatio, 0, maxExposureTime_, "gp: ++time!")) {
        return (true);
      }
    } else {
      // try bumping gain if exposure time is at limit
      if (changeGain(brightRatio, 0, maxGain_, "gp: ++gain!")) {
        return (true);
      }
    }
  }
  return (false);
}

bool MasterExposureController::updateExposureWithTimePriority(double brightRatio)
{
  if (brightRatio < 1) {  // image is too bright
    if (currentExposureTime_ > minExposureTime_) {
      // first try to shorten the exposure time
      if (changeExposure(brightRatio, minExposureTime_, maxExposureTime_, "tp: cut time!")) {
        return (true);
      }
    } else {
      // already have reached minimum exposure, try reducing gain
      if (currentGain_ > 0) {
        if (changeGain(brightRatio, 0, maxGain_, "tp: cut gain")) {
          return (true);
        }
      } else {
        // gain is already at zero, must reduce exposure below min
        if (changeExposure(brightRatio, 0, maxExposureTime_, "tp: cut time below min!")) {
          return (true);
        }
      }
    }
  } else {  // image is too dark
    // if current exposure time is below minimum, bump it
    if (currentExposureTime_ < minExposureTime_) {
      if (changeExposure(brightRatio, 0, minExposureTime_, "tp: bump time from min!")) {
        return (true);
      }
    } else {
      // next try bumping the gain
      if (currentGain_ < maxGain_) {
        if (changeGain(brightRatio, 0, maxGain_, "tp: bump gain")) {
          return (true);
        }
      } else {
        // already have max gain, must bump exposure time
        if (changeExposure(brightRatio, minExposureTime_, maxExposureTime_, "tp: ++time!")) {
          return (true);
        }
      }
    }
  }
  return (false);
}

bool MasterExposureController::updateExposure(double b)
{
  const double err_b = (brightnessTarget_ - b);
  // the current gain is higher than it should be, let's
  // set it to zero
  if (currentGain_ > maxGain_) {
    currentGain_ = 0;
    return (true);
  }

  // the current exposure is longer than it should be, let's
  // set it to a good value and retry
  if (currentExposureTime_ > maxExposureTime_) {
    currentExposureTime_ = maxExposureTime_;
    return (true);
  }
  if (fabs(err_b) <= brightnessTolerance_) {
    // no need to change anything!
    return (false);
  }
  // ratio between desired and actual brightness
  const double brightRatio = std::max(std::min(brightnessTarget_ / b, 10.0), 0.1);

  if (gainPriority_) {
    return (updateExposureWithGainPriority(brightRatio));
  } else {
    return (updateExposureWithTimePriority(brightRatio));
  }
  return (false);
}

void MasterExposureController::update(
  spinnaker_camera_driver::Camera * cam,
  const std::shared_ptr<const spinnaker_camera_driver::Image> & img)
{
  const int b = std::min(std::max(1, static_cast<int>(img->brightness_)), 255);
  // if the exposure parameters are not set yet, set them now
  if (currentExposureTime_ == 0) {
    currentExposureTime_ = static_cast<double>(img->exposureTime_);
  }
  if (currentGain_ == std::numeric_limits<float>::lowest()) {
    currentGain_ = img->gain_;
  }

  // check if the reported exposure and brightness settings
  // match ours. That means the changes have taken effect
  // on the driver side, and the brightness measurement can be
  // used right away.
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
    const double oldExposureTime = currentExposureTime_;
    const double oldGain = currentGain_;
    if (updateExposure(b)) {
      LOG_INFO(
        "bright " << b << " at time/gain: [" << oldExposureTime << " " << oldGain << "] new: ["
                  << currentExposureTime_ << " " << currentGain_ << "]");
      numFramesSkip_ = maxFramesSkip_;  // restart frame skipping
      const auto expName = cam->getPrefix() + exposureParameterName_;
      node_->set_parameter(rclcpp::Parameter(expName, currentExposureTime_));
      const auto gainName = cam->getPrefix() + gainParameterName_;
      node_->set_parameter(rclcpp::Parameter(gainName, currentGain_));
    }
  }
}

void MasterExposureController::addCamera(
  const std::shared_ptr<spinnaker_camera_driver::Camera> & cam)
{
  cameraName_ = cam->getName();
}

}  // namespace spinnaker_synchronized_camera_driver
