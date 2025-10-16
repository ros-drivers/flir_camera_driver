// -*-c++-*--------------------------------------------------------------------
// Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef SPINNAKER_CAMERA_DRIVER__LIFECYCLE_TYPES_HPP_
#define SPINNAKER_CAMERA_DRIVER__LIFECYCLE_TYPES_HPP_

#include <rclcpp/rclcpp.hpp>

#ifdef IMAGE_TRANSPORT_SUPPORTS_LIFECYCLE_NODE
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
using NodeType = rclcpp_lifecycle::LifecycleNode;
using LCState = rclcpp_lifecycle::State;
using CbReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

#else
using NodeType = rclcpp::Node;
#endif

#endif  // SPINNAKER_CAMERA_DRIVER__LIFECYCLE_TYPES_HPP_
