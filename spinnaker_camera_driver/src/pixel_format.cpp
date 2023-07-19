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

#include <spinnaker_camera_driver/pixel_format.hpp>
#include <string>
#include <unordered_map>

namespace spinnaker_camera_driver
{
namespace pixel_format
{
static const std::unordered_map<PixelFormat, std::string> fmt_2_string{
  {{PixelFormat::Mono8, "Mono8"},
   {PixelFormat::Mono10p, "Mono10p"},
   {PixelFormat::Mono10Packed, "Mono10Packed"},
   {PixelFormat::Mono12p, "Mono12p"},
   {PixelFormat::Mono12Packed, "Mono12Packed"},
   {PixelFormat::Mono16, "Mono16"},
   {PixelFormat::RGB8, "RGB8"},
   {PixelFormat::RGB8Packed, "RGB8Packed"},
   {PixelFormat::BayerRG8, "BayerRG8"},
   {PixelFormat::BayerRG10p, "BayerRG10p"},
   {PixelFormat::BayerRG10Packed, "BayerRG10Packed"},
   {PixelFormat::BayerRG12p, "BayerRG12p"},
   {PixelFormat::BayerRG12Packed, "BayerRG12Packed"},
   {PixelFormat::BayerRG16, "BayerRG16"},
   {PixelFormat::BayerGR8, "BayerGR8"},
   {PixelFormat::BayerGR16, "BayerGR16"},
   {PixelFormat::BayerGB8, "BayerGB8"},
   {PixelFormat::BayerGB16, "BayerGB16"},
   {PixelFormat::BayerBG8, "BayerBG8"},
   {PixelFormat::BayerBG16, "BayerBG16"},
   {PixelFormat::YUV411Packed, "YUV411Packed"},
   {PixelFormat::YUV422Packed, "YUV422Packed"},
   {PixelFormat::YUV444Packed, "YUV444Packed"},
   {PixelFormat::YCbCr8, "YCbCr8"},
   {PixelFormat::YCbCr422_8, "YCbCr422_8"},
   {PixelFormat::YCbCr411_8, "YCbCr411_8"},
   {PixelFormat::BGR8, "BGR8"},
   {PixelFormat::BGRa8, "BGRa8"}}};

static const std::unordered_map<std::string, PixelFormat> string_2_fmt{
  {{"Mono8", Mono8},
   {"Mono10p", Mono10p},
   {"Mono10Packed", Mono10Packed},
   {"Mono12p", Mono12p},
   {"Mono12Packed", Mono12Packed},
   {"Mono16", Mono16},
   {"RGB8", RGB8},
   {"RGB8Packed", RGB8Packed},
   {"BayerRG8", BayerRG8},
   {"BayerRG10p", BayerRG10p},
   {"BayerRG10Packed", BayerRG10Packed},
   {"BayerRG12p", BayerRG12p},
   {"BayerRG12Packed", BayerRG12Packed},
   {"BayerRG16", BayerRG16},
   {"BayerGR8", BayerGR8},
   {"BayerGR16", BayerGR16},
   {"BayerGB8", BayerGB8},
   {"BayerGB16", BayerGB16},
   {"BayerBG8", BayerBG8},
   {"BayerBG16", BayerBG16},
   {"YUV411Packed", YUV411Packed},
   {"YUV422Packed", YUV422Packed},
   {"YUV444Packed", YUV444Packed},
   {"YCbCr8", YCbCr8},
   {"YCbCr422_8", YCbCr422_8},
   {"YCbCr411_8", YCbCr411_8},
   {"BGR8", BGR8},
   {"BGRa8", BGRa8}}};

PixelFormat from_nodemap_string(const std::string pixFmt)
{
  auto it = string_2_fmt.find(pixFmt);
  return (it == string_2_fmt.end() ? INVALID : it->second);
}

std::string to_string(PixelFormat f)
{
  auto it = fmt_2_string.find(f);
  return (it == fmt_2_string.end() ? "INVALID" : it->second);
}

}  // namespace pixel_format
}  // namespace spinnaker_camera_driver
