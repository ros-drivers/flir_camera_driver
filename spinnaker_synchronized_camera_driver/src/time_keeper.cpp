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

#include <spinnaker_synchronized_camera_driver/logging.hpp>
#include <spinnaker_synchronized_camera_driver/synchronized_camera_driver.hpp>
#include <spinnaker_synchronized_camera_driver/time_keeper.hpp>

static rclcpp::Logger get_logger() { return (rclcpp::get_logger("cam_sync")); }

namespace spinnaker_synchronized_camera_driver
{
bool TimeKeeper::getTimeStamp(
  uint64_t hostTime, uint64_t, uint64_t frameId, size_t ninc, uint64_t * frameTime)
{
  if (lastHostTime_ == 0) {
    lastFrameId_ = frameId;
    lastHostTime_ = hostTime;
    return (false);
  }
  const int64_t gap = frameId - lastFrameId_;
  const int64_t dt64 = static_cast<int64_t>(hostTime) - static_cast<int64_t>(lastHostTime_);
  lastFrameId_ = frameId;
  lastHostTime_ = hostTime;

  numFramesDropped_ += std::max<int64_t>(0, gap - 1);
  numFramesIncomplete_ += ninc;
  if (gap > 0 && gap < 4) {  // ignore all but none, 1 or 2 frames dropped
    if (gap != 1) {
      LOG_WARN(name_ << " dropped " << gap - 1 << " frame(s)");
    }
    const double dt = dt64 * 1e-9 / static_cast<double>(gap);
    const bool gotTime = driver_->update(index_, hostTime, dt, frameTime);
    if (gotTime) {
      const double offset =
        (static_cast<int64_t>(hostTime) - static_cast<int64_t>(*frameTime)) * 1e-9;
      offsetSum_ += offset;
      // running variance is computed following this reference:
      // https://www.johndcook.com/blog/standard_deviation/
      if (numOffset_ == 0) {
        M_ = offset;
        S_ = 0;
      } else {
        const double M_km1 = M_;
        M_ += (offset - M_) / static_cast<double>(numOffset_ + 1);
        S_ += (offset - M_km1) * (offset - M_);
      }
      numOffset_++;
    }
    return (gotTime);
  } else {
    if (frameId != 0) {
      LOG_WARN(name_ << " skipping frame with frame id gap of " << gap);
    }
  }
  *frameTime = hostTime;
  return (false);
}

void TimeKeeper::clearStatistics()
{
  offsetSum_ = 0.0;
  numOffset_ = 0;
  numFramesDropped_ = 0;
  numFramesIncomplete_ = 0;
  S_ = 0;
  M_ = 0;
}

}  // namespace spinnaker_synchronized_camera_driver
