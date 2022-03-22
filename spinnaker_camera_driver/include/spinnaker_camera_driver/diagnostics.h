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

/*-*-C++-*-*/
/**
   @file diagnostics.h
   @author Michael Lowe
   @date August 1, 2018
   @brief Class to support ROS Diagnostics Aggregator for Spinnaker Camera

   @attention Copyright (C) 2018
*/

#ifndef SPINNAKER_CAMERA_DRIVER_DIAGNOSTICS_H
#define SPINNAKER_CAMERA_DRIVER_DIAGNOSTICS_H

#include "spinnaker_camera_driver/SpinnakerCamera.h"
#include "spinnaker_camera_driver/diagnostics.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/AddDiagnostics.h>
#include <ros/ros.h>
#include <bondcpp/bond.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace spinnaker_camera_driver
{
class DiagnosticsManager
{
public:
  DiagnosticsManager(const std::string name, const std::string serial,
                      std::shared_ptr<ros::Publisher> const& pub,
                      const ros::NodeHandle& nh);
  ~DiagnosticsManager();

  /*!
   * \brief Read the property of given parameters and push to aggregator
   *
   * Take all the collected parameters that were added read the values, then
   * publish them to the
   * allow the diagnostics aggreagtor to collect them
   * \param spinnaker the SpinnakerCamera object used for getting the parameters
   * from the spinnaker API
   */
  void processDiagnostics(SpinnakerCamera* spinnaker);

  /*!
   * \brief Add a diagnostic with name only (no warning checks)
   *
   * Allows the user to add an integer or float parameter without having to give
   * additional information.
   * User must specify the type they are getting
   * \param name is the name of the parameter as writting in the User Manual
   */
  template <typename T>
  void addDiagnostic(const Spinnaker::GenICam::gcstring name);

  /*!
   * \brief Add a diagnostic with warning checks
   *
   * Allows the user to add an integer or float parameter and values to check it
   * against. Anything outside
   * of these ranges will be considered an error.
   * \param name is the name of the parameter as writting in the User Manual
   */
  void addDiagnostic(const Spinnaker::GenICam::gcstring name, bool check_ranges = false,
                     std::pair<int, int> operational = std::make_pair(0, 0), int lower_bound = 0, int upper_bound = 0);
  void addDiagnostic(const Spinnaker::GenICam::gcstring name, bool check_ranges = false,
                     std::pair<float, float> operational = std::make_pair(0.0, 0.0), float lower_bound = 0,
                     float upper_bound = 0);

  void addAnalyzers();

private:
  /*
   * diagnostic_params is aData Structure to represent a parameter and its
   * bounds
   */
  template <typename T>
  struct diagnostic_params
  {
    Spinnaker::GenICam::gcstring parameter_name;  // This should be the same as written in the User Manual
    bool check_ranges;
    std::pair<T, T> operational_range;  // Normal operatinal range
    T warn_range_lower;
    T warn_range_upper;
  };

  /*!
   * \brief Function to push the diagnostic to the publisher
   *
   * Allows the user to add an integer or float parameter without having to give
   * additional information.
   * User must specify the type they are getting
   * \param param is the diagnostic parameter name and boundaries
   * \param value is the current value of the parameter requested from the
   * device
   */
  template <typename T>
  diagnostic_msgs::DiagnosticStatus getDiagStatus(const diagnostic_params<T>& param, const T value);

  // constuctor parameters
  std::string camera_name_;
  std::string serial_number_;
  std::shared_ptr<ros::Publisher> diagnostics_pub_;
  ros::NodeHandle nh_;
  std::shared_ptr<bond::Bond> bond_ = nullptr;

  // vectors to keep track of the items to publish
  std::vector<diagnostic_params<int>> integer_params_;
  std::vector<diagnostic_params<float>> float_params_;
  // Information about the device model, firmware, etc
  // TODO(mlowe): Allow these to be configured
  // clang-format off
  const std::vector<std::string> manufacturer_params_
  {
    "DeviceVendorName", "DeviceModelName", "SensorDescription", "DeviceFirmwareVersion"
  };
  // clang-format on
};
}  // namespace spinnaker_camera_driver

#endif  // SPINNAKER_CAMERA_DRIVER_DIAGNOSTICS_H
