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

#ifndef SPINNAKER_SYNCHRONIZED_CAMERA_DRIVER__TIME_KEEPER_HPP_
#define SPINNAKER_SYNCHRONIZED_CAMERA_DRIVER__TIME_KEEPER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <spinnaker_camera_driver/synchronizer.hpp>

namespace spinnaker_synchronized_camera_driver
{
class SynchronizedCameraDriver;  // forward decl
class TimeKeeper : public spinnaker_camera_driver::Synchronizer
{
public:
  explicit TimeKeeper(size_t idx, const std::string & name, SynchronizedCameraDriver * driver)
  : index_(idx), name_(name), driver_(driver)
  {
  }
  const std::string & getName() const { return (name_); }

  bool getTimeStamp(
    uint64_t hostTime, uint64_t imageTime, uint64_t frameId, size_t ninc, uint64_t * ft) override;

  double getOffsetAverage() const
  {
    return (numOffset_ != 0 ? offsetSum_ / static_cast<double>(numOffset_) : 0);
  }
  double getOffsetVariance() const
  {
    // running variance is computed following this reference:
    // https://www.johndcook.com/blog/standard_deviation/
    return (numOffset_ > 1 ? S_ / static_cast<double>(numOffset_ - 1) : 0);
  }
  int64_t getNumFramesDropped() const { return (numFramesDropped_); }
  size_t getNumFramesIncomplete() const { return (numFramesIncomplete_); }
  void clearStatistics();

private:
  size_t index_;
  std::string name_;
  SynchronizedCameraDriver * driver_{nullptr};
  uint64_t lastFrameId_{0};
  uint64_t lastHostTime_{0};
  int64_t numFramesDropped_{0};
  size_t numFramesIncomplete_{0};
  size_t numOffset_{0};
  double offsetSum_{0};
  double S_{0};
  double M_{0};
};
}  // namespace spinnaker_synchronized_camera_driver
#endif  // SPINNAKER_SYNCHRONIZED_CAMERA_DRIVER__TIME_KEEPER_HPP_
