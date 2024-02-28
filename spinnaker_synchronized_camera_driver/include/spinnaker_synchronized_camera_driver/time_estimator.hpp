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

#ifndef SPINNAKER_SYNCHRONIZED_CAMERA_DRIVER__TIME_ESTIMATOR_HPP_
#define SPINNAKER_SYNCHRONIZED_CAMERA_DRIVER__TIME_ESTIMATOR_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <list>

namespace spinnaker_synchronized_camera_driver
{
class TimeEstimator
{
public:
  TimeEstimator() = default;
  void initialize(uint64_t t0, double dt);
  bool update(size_t idx, uint64_t t, uint64_t * frameTime);

private:
  class FrameTime
  {
  public:
    explicit FrameTime(int64_t frameTime) : frameTime_(frameTime), sumFrameTime_(0), numFrames_(0)
    {
    }
    int64_t getFrameTime() const { return (frameTime_); }
    void addTime(const uint64_t t)
    {
      sumFrameTime_ += static_cast<double>(t) * 1e-9;
      numFrames_++;
    }
    bool isValid() const { return (numFrames_ > 0); }
    double getAverageFrameTime() const
    {
      return (numFrames_ > 0 ? sumFrameTime_ / static_cast<double>(numFrames_) : 0);
    }
    size_t getNumberOfFrames() const { return (numFrames_); }

  private:
    int64_t frameTime_{0};
    double sumFrameTime_{0};
    size_t numFrames_{0};
  };
  bool getTimeFromList(uint64_t t, uint64_t * T);
  void updateKalman(double z);
  int64_t predict();
  using Vec2d = std::array<double, 2>;
  using Mat2x2d = std::array<Vec2d, 2>;
  Vec2d x_;
  Mat2x2d P_;
  double R_{0};
  double Q_{0};
  uint64_t T0_{0};
  std::list<FrameTime> frameTimes_;
};
}  // namespace spinnaker_synchronized_camera_driver
#endif  // SPINNAKER_SYNCHRONIZED_CAMERA_DRIVER__TIME_ESTIMATOR_HPP_
