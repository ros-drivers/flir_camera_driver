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

#ifndef SPINNAKER_CAMERA_DRIVER__IP_ADDRESS_HPP_
#define SPINNAKER_CAMERA_DRIVER__IP_ADDRESS_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace spinnaker_camera_driver
{
namespace ip_address
{
/// Parses a dotted-quad IPv4 address such as "192.168.1.64". Returns false if the text is not one.
bool parse(const std::string & text, uint32_t * address);

/// Formats an IPv4 address as a dotted quad, most significant byte first.
std::string format(uint32_t address);

/// Returns the index of the entry of `candidates` that is the IPv4 address `target`, or -1.
/// Candidates that do not parse, such as the empty text of a camera without an IP, never match.
int find(const std::vector<std::string> & candidates, uint32_t target);
}  // namespace ip_address
}  // namespace spinnaker_camera_driver
#endif  // SPINNAKER_CAMERA_DRIVER__IP_ADDRESS_HPP_
