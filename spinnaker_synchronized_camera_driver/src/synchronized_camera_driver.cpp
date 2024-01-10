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

#include <image_transport/image_transport.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <spinnaker_synchronized_camera_driver/logging.hpp>
#include <spinnaker_synchronized_camera_driver/synchronized_camera_driver.hpp>
#include <spinnaker_synchronized_camera_driver/time_estimator.hpp>

namespace spinnaker_synchronized_camera_driver
{

SynchronizedCameraDriver::SynchronizedCameraDriver(const rclcpp::NodeOptions & options)
: Node("sync_cam_driver", options), timeEstimator_(new TimeEstimator())
{
  imageTransport_ = std::make_shared<image_transport::ImageTransport>(
    std::shared_ptr<SynchronizedCameraDriver>(this, [](auto *) {}));
  createCameras();
  // start cameras only when all synchronizer state has been set up!
  for (auto & c : cameras_) {
    c.second->start();
  }

  statusTimer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Duration(5, 0),
    std::bind(&SynchronizedCameraDriver::printStatus, this));
}

SynchronizedCameraDriver::~SynchronizedCameraDriver()
{
  if (!statusTimer_->is_canceled()) {
    statusTimer_->cancel();
  }
}

void SynchronizedCameraDriver::printStatus()
{
  if (numUpdatesReceived_ < numUpdatesRequired_) {
    LOG_INFO("waiting for accurate frequency, current estimate: " << 1.0 / avgFrameInterval_);
    return;
  }

  struct TKInfo
  {
    explicit TKInfo(const std::string & n, double off, double jit, int64_t d)
    : name(n), offset(off), jitter(jit), dropped(d)
    {
    }
    std::string name;
    double offset;
    double jitter;
    int64_t dropped;
  };
  std::vector<TKInfo> tki;
  double dt = 0;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    dt = avgFrameInterval_;
    for (auto & tk : timeKeepers_) {
      tki.push_back(TKInfo(
        tk->getName(), tk->getOffsetAverage() / dt, std::sqrt(tk->getOffsetVariance()) / dt,
        tk->getNumFramesDropped()));
      tk->clearStatistics();
    }
  }
  LOG_INFO_FMT("------ frequency: %10.3f Hz", 1.0 / dt);
  LOG_INFO_FMT("%-10s %5s %10s %10s", "camera", "drop", "offset", "jitter");
  for (auto & tk : tki) {
    LOG_INFO_FMT(
      "%-10s %5ld %9.2f%% %9.2f%%", tk.name.c_str(), tk.dropped, tk.offset * 100, tk.jitter * 100);
  }
}

void SynchronizedCameraDriver::createCameras()
{
  using svec = std::vector<std::string>;
  const svec cameras = this->declare_parameter<svec>("cameras", svec());
  if (cameras.empty()) {
    BOMB_OUT("no cameras configured for synchronized driver!");
  }

  for (size_t i = 0; i < cameras.size(); i++) {
    const auto & c = cameras[i];
    auto cam =
      std::make_shared<spinnaker_camera_driver::Camera>(this, imageTransport_.get(), c, false);
    cameras_.insert({c, cam});
    timeKeepers_.push_back(std::make_shared<TimeKeeper>(i, c, this));
    cam->setSynchronizer(timeKeepers_.back());
  }
  numUpdatesRequired_ = cameras.size() * 3;
}

bool SynchronizedCameraDriver::update(
  size_t idx, uint64_t hostTime, double dt, uint64_t * frameTime)
{
  std::unique_lock<std::mutex> lock(mutex_);
  constexpr double NUM_FRAMES_TO_AVG = 20.0;
  constexpr double alpha = 1.0 / NUM_FRAMES_TO_AVG;
  avgFrameInterval_ =
    (avgFrameInterval_ == 0) ? dt : (avgFrameInterval_ * (1.0 - alpha) + alpha * dt);
  if (numUpdatesReceived_ < numUpdatesRequired_) {
    numUpdatesReceived_++;
    if (numUpdatesReceived_ >= numUpdatesRequired_) {
      timeEstimator_->initialize(hostTime, avgFrameInterval_);
    }
    *frameTime = hostTime;
    return (true);
  }
  const bool gotTime = timeEstimator_->update(idx, hostTime, frameTime);
  return (gotTime);
}

}  // namespace spinnaker_synchronized_camera_driver

RCLCPP_COMPONENTS_REGISTER_NODE(spinnaker_synchronized_camera_driver::SynchronizedCameraDriver)
