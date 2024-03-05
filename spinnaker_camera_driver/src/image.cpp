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

#include <spinnaker_camera_driver/image.hpp>
namespace spinnaker_camera_driver
{
Image::Image(
  uint64_t t, int16_t brightness, uint32_t et, uint32_t maxEt, float gain, int64_t imgT,
  size_t imageSize, int status, const void * data, size_t w, size_t h, size_t stride,
  size_t bitsPerPixel, size_t numChan, uint64_t frameId, pixel_format::PixelFormat pixFmt,
  size_t ninc)
: time_(t),
  brightness_(brightness),
  exposureTime_(et),
  maxExposureTime_(maxEt),
  gain_(gain),
  imageTime_(imgT),
  imageSize_(imageSize),
  imageStatus_(status),
  data_(data),
  width_(w),
  height_(h),
  stride_(stride),
  bitsPerPixel_(bitsPerPixel),
  numChan_(numChan),
  frameId_(frameId),
  pixelFormat_(pixFmt),
  numIncomplete_(ninc)
{
}
}  // namespace spinnaker_camera_driver
