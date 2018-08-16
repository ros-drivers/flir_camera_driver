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

#include <utility>
#include <string>

namespace spinnaker_camera_driver
{
DiagnosticsManager::DiagnosticsManager(const std::string name, const std::string serial,
                                       std::shared_ptr<ros::Publisher> const& pub)
  : camera_name_(name), serial_number_(serial), diagnostics_pub_(pub)
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

template <typename T>
diagnostic_msgs::DiagnosticStatus DiagnosticsManager::getDiagStatus(const diagnostic_params<T>& param, const T value)
{
  diagnostic_msgs::KeyValue kv;
  kv.key = param.parameter_name;
  kv.value = std::to_string(value);

  diagnostic_msgs::DiagnosticStatus diag_status;
  diag_status.values.push_back(kv);
  diag_status.name = "Spinnaker " + Spinnaker::GenICam::gcstring(camera_name_.c_str()) + " " + param.parameter_name;
  diag_status.hardware_id = serial_number_;

  // Determine status level
  if (!param.check_ranges || (value > param.operational_range.first && value <= param.operational_range.second))
  {
    diag_status.level = 0;
    diag_status.message = "OK";
  }
  else if (value >= param.warn_range_lower && value <= param.warn_range_upper)
  {
    diag_status.level = 1;
    diag_status.message = "WARNING";
  }
  else
  {
    diag_status.level = 2;
    diag_status.message = "ERROR";
  }

  return diag_status;
}

void DiagnosticsManager::processDiagnostics(SpinnakerCamera* spinnaker)
{
  diagnostic_msgs::DiagnosticArray diag_array;

  // Manufacturer Info
  diagnostic_msgs::DiagnosticStatus diag_manufacture_info;
  diag_manufacture_info.name = "Spinnaker " + camera_name_ + " Manufacture Info";
  diag_manufacture_info.hardware_id = serial_number_;

  for (const std::string param : manufacturer_params_)
  {
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
    Spinnaker::GenApi::CFloatPtr float_ptr =
        static_cast<Spinnaker::GenApi::CFloatPtr>(spinnaker->readProperty(param.parameter_name));

    float float_value = float_ptr->GetValue(true);

    diagnostic_msgs::DiagnosticStatus diag_status = getDiagStatus(param, float_value);
    diag_array.status.push_back(diag_status);
  }

  // Int based parameters
  for (const diagnostic_params<int>& param : integer_params_)
  {
    Spinnaker::GenApi::CIntegerPtr integer_ptr =
        static_cast<Spinnaker::GenApi::CIntegerPtr>(spinnaker->readProperty(param.parameter_name));

    int int_value = integer_ptr->GetValue(true);
    diagnostic_msgs::DiagnosticStatus diag_status = getDiagStatus(param, int_value);
    diag_array.status.push_back(diag_status);
  }

  diagnostics_pub_->publish(diag_array);
}
}  // namespace spinnaker_camera_driver
