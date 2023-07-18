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

// NOTE: much of this code is adapted from the Spinnaker examples,
// in particular NodeMapInfo.cpp
//
//

#include "genicam_utils.hpp"

#include <iostream>
#include <sstream>

using Spinnaker::GenApi::CCategoryPtr;
using Spinnaker::GenApi::CNodePtr;
using Spinnaker::GenApi::FeatureList_t;
using Spinnaker::GenApi::INodeMap;
using Spinnaker::GenApi::intfICategory;
using Spinnaker::GenICam::gcstring;

namespace spinnaker_camera_driver
{
namespace genicam_utils
{
template <class T>
static bool is_readable(T ptr)
{
  return (Spinnaker::GenApi::IsAvailable(ptr) && Spinnaker::GenApi::IsReadable(ptr));
}

void get_nodemap_as_string(std::stringstream & ss, Spinnaker::CameraPtr cam)
{
  gcstring s = cam->GetGuiXml();
  ss << s;
}

static CNodePtr find_node(const std::string & path, CNodePtr & node, bool debug)
{
  // split off first part
  auto pos = path.find("/");
  const std::string token = path.substr(0, pos);  // first part of it
  if (node->GetPrincipalInterfaceType() != intfICategory) {
    std::cerr << "no category node: " << node->GetName() << " vs " << path << std::endl;
    return (NULL);
  }

  CCategoryPtr catNode = static_cast<CCategoryPtr>(node);
  gcstring displayName = catNode->GetDisplayName();
  gcstring name = catNode->GetName();
  FeatureList_t features;
  catNode->GetFeatures(features);
  if (debug) {
    std::cout << "parsing: " << name << " with features: " << features.size() << std::endl;
  }
  for (auto it = features.begin(); it != features.end(); ++it) {
    CNodePtr childNode = *it;
    if (debug) {
      std::cout << "checking child: " << childNode->GetName() << " vs " << token << std::endl;
    }
    if (std::string(childNode->GetName().c_str()) == token) {
      if (is_readable(childNode)) {
        if (pos == std::string::npos) {  // no slash in name, found leaf node
          return (childNode);
        } else {
          const std::string rest = path.substr(pos + 1);
          return (find_node(rest, childNode, debug));
        }
      }
    }
  }
  if (debug) {
    std::cerr << "driver: node not found: " << path << std::endl;
  }
  return (CNodePtr(NULL));
}

CNodePtr find_node(const std::string & path, Spinnaker::CameraPtr cam, bool debug)
{
  INodeMap & appLayerNodeMap = cam->GetNodeMap();
  CNodePtr rootNode = appLayerNodeMap.GetNode("Root");
  CNodePtr retNode = find_node(path, rootNode, debug);
  return (retNode);
}
}  // namespace genicam_utils
}  // namespace spinnaker_camera_driver
