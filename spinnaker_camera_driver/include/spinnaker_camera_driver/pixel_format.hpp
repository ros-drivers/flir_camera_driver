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

#ifndef SPINNAKER_CAMERA_DRIVER__PIXEL_FORMAT_HPP_
#define SPINNAKER_CAMERA_DRIVER__PIXEL_FORMAT_HPP_

#include <memory>
#include <string>

namespace spinnaker_camera_driver
{
namespace pixel_format
{
enum PixelFormat {
  INVALID = 0,
  Mono8,
  Mono10p,
  Mono10Packed,
  Mono12p,
  Mono12Packed,
  Mono16,
  RGB8,
  RGB8Packed,
  BayerRG8,
  BayerRG10p,
  BayerRG10Packed,
  BayerRG12p,
  BayerRG12Packed,
  BayerRG16,
  BayerGR8,
  BayerGR16,
  BayerGB8,
  BayerGB16,
  BayerBG8,
  BayerBG16,
  YUV411Packed,
  YUV422Packed,
  YUV444Packed,
  YCbCr8,
  YCbCr422_8,
  YCbCr411_8,
  BGR8,
  BGRa8
};
std::string to_string(PixelFormat f);
PixelFormat from_nodemap_string(const std::string pixFmt);
}  // namespace pixel_format
}  // namespace spinnaker_camera_driver
#endif  // SPINNAKER_CAMERA_DRIVER__PIXEL_FORMAT_HPP_
