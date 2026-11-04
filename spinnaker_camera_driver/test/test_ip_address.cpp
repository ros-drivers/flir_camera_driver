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

#include <gtest/gtest.h>

#include <spinnaker_camera_driver/ip_address.hpp>
#include <string>
#include <vector>

namespace ip = spinnaker_camera_driver::ip_address;

TEST(IpAddress, ParsesDottedQuad)
{
  uint32_t address = 0;
  ASSERT_TRUE(ip::parse("192.168.1.64", &address));
  EXPECT_EQ(0xC0A80140u, address);
  ASSERT_TRUE(ip::parse("0.0.0.0", &address));
  EXPECT_EQ(0u, address);
  ASSERT_TRUE(ip::parse("255.255.255.255", &address));
  EXPECT_EQ(0xFFFFFFFFu, address);
}

TEST(IpAddress, RejectsMalformedText)
{
  uint32_t address = 0;
  for (const std::string text :
       {"", "192.168.1", "192.168.1.64.5", "192.168.1.256", "192.168.1.x", " 192.168.1.64",
        "192.168.1.64 ", "20435008", "-1.0.0.0"}) {
    EXPECT_FALSE(ip::parse(text, &address)) << text;
  }
}

TEST(IpAddress, FormatInvertsParse)
{
  uint32_t address = 0;
  ASSERT_TRUE(ip::parse("10.0.5.200", &address));
  EXPECT_EQ("10.0.5.200", ip::format(address));
}

TEST(IpAddress, FindsTheMatchingCandidate)
{
  uint32_t target = 0;
  ASSERT_TRUE(ip::parse("192.168.1.65", &target));
  EXPECT_EQ(1, ip::find({"192.168.1.64", "192.168.1.65", "192.168.1.66"}, target));
}

TEST(IpAddress, FindSkipsCamerasWithoutAnIp)
{
  uint32_t target = 0;
  ASSERT_TRUE(ip::parse("192.168.1.65", &target));
  EXPECT_EQ(2, ip::find({"", "not an ip", "192.168.1.65"}, target));
  EXPECT_EQ(-1, ip::find({"", "192.168.1.64"}, target));
  EXPECT_EQ(-1, ip::find({}, target));
}
