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

#ifndef GENICAM_UTILS_HPP_
#define GENICAM_UTILS_HPP_

#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>

#include <sstream>
#include <string>

namespace spinnaker_camera_driver
{
namespace genicam_utils
{
void get_nodemap_as_string(std::stringstream & ss, Spinnaker::CameraPtr cam);
Spinnaker::GenApi::CNodePtr find_node(
  const std::string & path, Spinnaker::CameraPtr cam, bool debug);
}  // namespace genicam_utils
}  // namespace spinnaker_camera_driver

#endif  // GENICAM_UTILS_HPP_
