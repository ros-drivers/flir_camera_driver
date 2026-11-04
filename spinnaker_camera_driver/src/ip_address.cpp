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

#include <cstdio>
#include <spinnaker_camera_driver/ip_address.hpp>
#include <string>
#include <vector>

namespace spinnaker_camera_driver
{
namespace ip_address
{
bool parse(const std::string & text, uint32_t * address)
{
  uint32_t value = 0;
  size_t pos = 0;
  for (int octet = 0; octet < 4; octet++) {
    if (octet > 0) {
      if (pos >= text.size() || text[pos] != '.') {
        return false;
      }
      pos++;
    }
    const size_t first = pos;
    uint32_t number = 0;
    while (pos < text.size() && text[pos] >= '0' && text[pos] <= '9') {
      number = number * 10 + static_cast<uint32_t>(text[pos] - '0');
      if (number > 255) {
        return false;
      }
      pos++;
    }
    if (pos == first) {
      return false;
    }
    value = (value << 8) | number;
  }
  if (pos != text.size()) {
    return false;
  }
  *address = value;
  return true;
}

std::string format(uint32_t address)
{
  char buf[16];
  std::snprintf(
    buf, sizeof(buf), "%u.%u.%u.%u", (address >> 24) & 0xFF, (address >> 16) & 0xFF,
    (address >> 8) & 0xFF, address & 0xFF);
  return std::string(buf);
}

int find(const std::vector<std::string> & candidates, uint32_t target)
{
  for (size_t i = 0; i < candidates.size(); i++) {
    uint32_t candidate;
    if (parse(candidates[i], &candidate) && candidate == target) {
      return static_cast<int>(i);
    }
  }
  return -1;
}
}  // namespace ip_address
}  // namespace spinnaker_camera_driver
