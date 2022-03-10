/**
Software License Agreement (BSD)

\file      diagnostics.cpp
\authors   Michael Lowe <michael@ascent.ai>
\copyright Copyright (c) 2018, Ascent Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "spinnaker_camera_driver/diagnostics.h"

#include <memory>
#include <string>
#include <utility>

namespace spinnaker_camera_driver
{
DiagnosticsManager::DiagnosticsManager(const std::string name, const std::string serial,
                                       std::shared_ptr<ros::Publisher> const& pub,
                                       const ros::NodeHandle& nh)
  : camera_name_(name), serial_number_(serial), diagnostics_pub_(pub), nh_(nh)
{
}

DiagnosticsManager::~DiagnosticsManager()
{
}

template <typename T>
void DiagnosticsManager::addDiagnostic(const Spinnaker::GenICam::gcstring name)
{
  T first = 0;
  T second = 0;
  // Call the overloaded function (use the pair to determine which one)
  addDiagnostic(name, false, std::make_pair(first, second));
}

template void DiagnosticsManager::addDiagnostic<int>(const Spinnaker::GenICam::gcstring name);

template void DiagnosticsManager::addDiagnostic<float>(const Spinnaker::GenICam::gcstring name);

void DiagnosticsManager::addDiagnostic(const Spinnaker::GenICam::gcstring name, bool check_ranges,
                                       std::pair<int, int> operational, int lower_bound, int upper_bound)
{
  diagnostic_params<int> param{ name, check_ranges, operational, lower_bound, upper_bound };
  integer_params_.push_back(param);
}

void DiagnosticsManager::addDiagnostic(const Spinnaker::GenICam::gcstring name, bool check_ranges,
                                       std::pair<float, float> operational, float lower_bound, float upper_bound)
{
  diagnostic_params<float> param{ name, check_ranges, operational, lower_bound, upper_bound };
  float_params_.push_back(param);
}

void DiagnosticsManager::addAnalyzers()
{
  // Get Namespace
  std::string node_name = ros::this_node::getName().substr(1);
  std::string node_namespace = ros::this_node::getNamespace();
  std::string node_prefix = "";
  std::string node_path;
  std::string node_id = node_name;

  // Create "Fake" Namespace for Diagnostics
  if (node_namespace == "/")
  {
    node_namespace = ros::this_node::getName();
    node_prefix = ros::this_node::getName() + "/";
  }

  // Sanitize Node ID
  size_t pos = node_id.find("/");
  while (pos != std::string::npos)
  {
    node_id.replace(pos, 1, "_");
    pos = node_id.find("/");
  }

  // Sanitize Node Path
  node_path = node_id;
  pos = node_path.find("_");
  while (pos != std::string::npos)
  {
    node_path.replace(pos, 1, " ");
    pos = node_path.find("_");
  }

  // GroupAnalyzer Parameters
  if (!ros::param::has(node_prefix + "analyzers/spinnaker/path"))
  {
    ros::param::set(node_prefix + "analyzers/spinnaker/path", "Spinnaker");
    ros::param::set(node_prefix + "analyzers/spinnaker/type", "diagnostic_aggregator/AnalyzerGroup");
  }

  // Analyzer Parameters
  std::string analyzerPath = node_prefix + "analyzers/spinnaker/analyzers/" + node_id;
  if (!ros::param::has(analyzerPath + "/path"))
  {
    ros::param::set(analyzerPath + "/path", node_path);
    ros::param::set(analyzerPath + "/type", "diagnostic_aggregator/GenericAnalyzer");
    ros::param::set(analyzerPath + "/startswith", node_name);
    ros::param::set(analyzerPath + "/remove_prefix", node_name);
  }

  // Bond to Diagnostics Aggregator
  if (bond_ == nullptr)
  {
    bond_ = std::shared_ptr<bond::Bond>(new bond::Bond("/diagnostics_agg/bond" + node_namespace, node_namespace));
  }
  else if (!bond_->isBroken())
  {
    return;
  }
  bond_->setConnectTimeout(120);

  // Add Diagnostics
  diagnostic_msgs::AddDiagnostics srv;
  srv.request.load_namespace = node_namespace;
  if (!ros::service::waitForService("/diagnostics_agg/add_diagnostics", 1000))
  {
    return;
  }
  bond_->start();
  ros::service::call("/diagnostics_agg/add_diagnostics", srv);
}

template <typename T>
diagnostic_msgs::DiagnosticStatus DiagnosticsManager::getDiagStatus(const diagnostic_params<T>& param, const T value)
{
  std::string node_name = ros::this_node::getName().substr(1);
  diagnostic_msgs::KeyValue kv;
  kv.key = param.parameter_name;
  kv.value = std::to_string(value);

  diagnostic_msgs::DiagnosticStatus diag_status;
  diag_status.values.push_back(kv);
  diag_status.name = node_name + ":" + std::string(param.parameter_name.c_str());
  diag_status.hardware_id = serial_number_;

  // Determine status level
  if (!param.check_ranges || (value > param.operational_range.first && value <= param.operational_range.second))
  {
    diag_status.level = 0;
    diag_status.message = "OK: " + std::string(param.parameter_name)
                          + " performing in expected operational range.";
  }
  else if (value >= param.warn_range_lower && value <= param.warn_range_upper)
  {
    diag_status.level = 1;
    diag_status.message = "WARNING: " + std::string(param.parameter_name.c_str())
                          + " is not in expected operational range.";
  }
  else
  {
    diag_status.level = 2;
    diag_status.message = "ERROR: " + std::string(param.parameter_name.c_str())
                          + " is in critical operation range.";
  }
  // Warning Range
  kv.key = "Warning Range";
  kv.value = "[" + std::to_string(param.warn_range_lower) + ", "
              + std::to_string(param.warn_range_upper) + "]";
  diag_status.values.push_back(kv);

  // Operational Range
  kv.key = "Operational Range";
  kv.value = "[" + std::to_string(param.operational_range.first) + ", "
              + std::to_string(param.operational_range.second) + "]";
  diag_status.values.push_back(kv);

  return diag_status;
}

void DiagnosticsManager::processDiagnostics(SpinnakerCamera* spinnaker)
{
  std::string node_name = ros::this_node::getName().substr(1);
  diagnostic_msgs::DiagnosticArray diag_array;

  // Manufacturer Info
  diagnostic_msgs::DiagnosticStatus diag_manufacture_info;
  diag_manufacture_info.name = node_name + ": Manufacture Info";
  diag_manufacture_info.hardware_id = serial_number_;

  for (const std::string param : manufacturer_params_)
  {
    // Check if Readable
    if (!spinnaker->readableProperty(Spinnaker::GenICam::gcstring(param.c_str())))
    {
      diagnostic_msgs::KeyValue kv;
      kv.key = param;
      kv.value = "Property not Available and Readable";
      diag_manufacture_info.values.push_back(kv);
      continue;
    }

    // Write if Readable
    Spinnaker::GenApi::CStringPtr string_ptr = static_cast<Spinnaker::GenApi::CStringPtr>(
        spinnaker->readProperty(Spinnaker::GenICam::gcstring(param.c_str())));

    diagnostic_msgs::KeyValue kv;
    kv.key = param;
    kv.value = string_ptr->GetValue(true);
    diag_manufacture_info.values.push_back(kv);
  }

  diag_array.status.push_back(diag_manufacture_info);

  // Float based parameters
  for (const diagnostic_params<float>& param : float_params_)
  {
    // Check if Readable
    if (!spinnaker->readableProperty(Spinnaker::GenICam::gcstring(param.parameter_name)))
    {
      diagnostic_msgs::KeyValue kv;
      kv.key = param.parameter_name;
      kv.value = "Property not Available and Readable";
      diag_manufacture_info.values.push_back(kv);
      continue;
    }

    Spinnaker::GenApi::CFloatPtr float_ptr =
        static_cast<Spinnaker::GenApi::CFloatPtr>(spinnaker->readProperty(param.parameter_name));

    float float_value = float_ptr->GetValue(true);

    diagnostic_msgs::DiagnosticStatus diag_status = getDiagStatus(param, float_value);
    diag_array.status.push_back(diag_status);
  }

  // Int based parameters
  for (const diagnostic_params<int>& param : integer_params_)
  {
    // Check if Readable
    if (!spinnaker->readableProperty(Spinnaker::GenICam::gcstring(param.parameter_name)))
    {
      diagnostic_msgs::KeyValue kv;
      kv.key = param.parameter_name;
      kv.value = "Property not Available and Readable";
      diag_manufacture_info.values.push_back(kv);
      continue;
    }

    Spinnaker::GenApi::CIntegerPtr integer_ptr =
        static_cast<Spinnaker::GenApi::CIntegerPtr>(spinnaker->readProperty(param.parameter_name));

    int int_value = integer_ptr->GetValue(true);
    diagnostic_msgs::DiagnosticStatus diag_status = getDiagStatus(param, int_value);
    diag_array.status.push_back(diag_status);
  }

  diagnostics_pub_->publish(diag_array);
}
}  // namespace spinnaker_camera_driver
