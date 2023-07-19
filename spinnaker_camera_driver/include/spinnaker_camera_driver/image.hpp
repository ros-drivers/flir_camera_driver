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

#ifndef SPINNAKER_CAMERA_DRIVER__IMAGE_HPP_
#define SPINNAKER_CAMERA_DRIVER__IMAGE_HPP_

#include <memory>
#include <spinnaker_camera_driver/pixel_format.hpp>

namespace spinnaker_camera_driver
{
class Image
{
public:
  Image(
    uint64_t t, int16_t brightness, uint32_t et, uint32_t maxEt, float gain, int64_t imgT,
    size_t imageSize, int status, const void * data, size_t w, size_t h, size_t stride,
    size_t bitsPerPixel, size_t numChan, uint64_t frameId, pixel_format::PixelFormat pixFmt);

  // ----- variables --
  uint64_t time_;
  int16_t brightness_;
  uint32_t exposureTime_;
  uint32_t maxExposureTime_;
  float gain_;
  int64_t imageTime_;
  size_t imageSize_;
  int imageStatus_;
  const void * data_;
  size_t width_;
  size_t height_;
  size_t stride_;  // in bytes
  size_t bitsPerPixel_;
  size_t numChan_;
  uint64_t frameId_;
  pixel_format::PixelFormat pixelFormat_;

private:
};
typedef std::shared_ptr<Image> ImagePtr;
typedef std::shared_ptr<const Image> ImageConstPtr;
}  // namespace spinnaker_camera_driver
#endif  // SPINNAKER_CAMERA_DRIVER__IMAGE_HPP_
