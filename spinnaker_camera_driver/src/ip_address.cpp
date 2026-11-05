// -*-c++-*--------------------------------------------------------------------
// Copyright 2026 David Tarazi <davidtarazi@icloud.com>
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

#include <arpa/inet.h>

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
  in_addr parsed;
  if (1 != inet_pton(AF_INET, text.c_str(), &parsed)) {
    return false;
  }
  *address = ntohl(parsed.s_addr);
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
