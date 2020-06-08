/**
Software License Agreement (BSD)

\file      set_property.h
\authors   Michael Hosmar <mhosmar@clearpathrobotics.com>
\copyright Copyright (c) 2018, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef SPINNAKER_CAMERA_DRIVER_SET_PROPERTY_H
#define SPINNAKER_CAMERA_DRIVER_SET_PROPERTY_H

// Spinnaker SDK
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include <string>

namespace spinnaker_camera_driver
{
inline bool setProperty(Spinnaker::GenApi::INodeMap* node_map, const std::string& property_name,
                        const std::string& entry_name)
{
  // *** NOTES ***
  // Enumeration nodes are slightly more complicated to set than other
  // nodes. This is because setting an enumeration node requires working
  // with two nodes instead of the usual one.
  //
  // As such, there are a number of steps to setting an enumeration node:
  // retrieve the enumeration node from the nodemap, retrieve the desired
  // entry node from the enumeration node, retrieve the integer value from
  // the entry node, and set the new value of the enumeration node with
  // the integer value from the entry node.
  Spinnaker::GenApi::CEnumerationPtr enumerationPtr = node_map->GetNode(property_name.c_str());

  if (!Spinnaker::GenApi::IsImplemented(enumerationPtr))
  {
    ROS_ERROR_STREAM("[SpinnakerCamera]: ("
                     << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue()
                     << ") Enumeration name " << property_name << " not "
                                                                  "implemented.");
    return false;
  }

  if (Spinnaker::GenApi::IsAvailable(enumerationPtr))
  {
    if (Spinnaker::GenApi::IsWritable(enumerationPtr))
    {
      Spinnaker::GenApi::CEnumEntryPtr enumEmtryPtr = enumerationPtr->GetEntryByName(entry_name.c_str());

      if (Spinnaker::GenApi::IsAvailable(enumEmtryPtr))
      {
        if (Spinnaker::GenApi::IsReadable(enumEmtryPtr))
        {
          enumerationPtr->SetIntValue(enumEmtryPtr->GetValue());

          ROS_INFO_STREAM("[SpinnakerCamera]: ("
                          << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue()
                          << ") " << property_name << " set to " << enumerationPtr->GetCurrentEntry()->GetSymbolic()
                          << ".");

          return true;
        }
        else
        {
          ROS_WARN_STREAM("[SpinnakerCamera]: ("
                          << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue()
                          << ") Entry name " << entry_name << " not writable.");
        }
      }
      else
      {
        ROS_WARN_STREAM("[SpinnakerCamera]: ("
                        << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue()
                        << ") Entry name " << entry_name << " not available.");

        ROS_WARN("Available:");
        Spinnaker::GenApi::NodeList_t entries;
        enumerationPtr->GetEntries(entries);
        for (auto& entry : entries)
        {
          auto enumEntry = dynamic_cast<Spinnaker::GenApi::IEnumEntry*>(entry);
          if (enumEntry && Spinnaker::GenApi::IsAvailable(entry))
            ROS_WARN_STREAM(" - " << entry->GetName() << " (display " << entry->GetDisplayName() << ", symbolic "
                                  << enumEntry->GetSymbolic() << ")");
        }
      }
    }
    else
    {
      ROS_WARN_STREAM("[SpinnakerCamera]: ("
                      << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue()
                      << ") Enumeration " << property_name << " not writable.");
    }
  }
  else
  {
    ROS_WARN_STREAM("[SpinnakerCamera]: ("
                    << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue()
                    << ") Enumeration " << property_name << " not available.");
  }
  return false;
}

inline bool setProperty(Spinnaker::GenApi::INodeMap* node_map, const std::string& property_name, const float& value)
{
  Spinnaker::GenApi::CFloatPtr floatPtr = node_map->GetNode(property_name.c_str());

  if (!Spinnaker::GenApi::IsImplemented(floatPtr))
  {
    ROS_ERROR_STREAM("[SpinnakerCamera]: ("
                     << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue()
                     << ") Feature name " << property_name << " not implemented.");
    return false;
  }
  if (Spinnaker::GenApi::IsAvailable(floatPtr))
  {
    if (Spinnaker::GenApi::IsWritable(floatPtr))
    {
      float temp_value = value;
      if (temp_value > floatPtr->GetMax())
        temp_value = floatPtr->GetMax();
      else if (temp_value < floatPtr->GetMin())
        temp_value = floatPtr->GetMin();
      floatPtr->SetValue(temp_value);
      ROS_INFO_STREAM("[SpinnakerCamera]: ("
                      << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") "
                      << property_name << " set to " << floatPtr->GetValue() << ".");
      return true;
    }
    else
    {
      ROS_WARN_STREAM("[SpinnakerCamera]: ("
                      << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue()
                      << ") Feature " << property_name << " not writable.");
    }
  }
  else
  {
    ROS_WARN_STREAM("[SpinnakerCamera]: ("
                    << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue()
                    << ") Feature " << property_name << " not available.");
  }
  return false;
}

inline bool setProperty(Spinnaker::GenApi::INodeMap* node_map, const std::string& property_name, const bool& value)
{
  Spinnaker::GenApi::CBooleanPtr boolPtr = node_map->GetNode(property_name.c_str());
  if (!Spinnaker::GenApi::IsImplemented(boolPtr))
  {
    ROS_ERROR_STREAM("[SpinnakerCamera]: ("
                     << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue()
                     << ") Feature name " << property_name << " not implemented.");
    return false;
  }
  if (Spinnaker::GenApi::IsAvailable(boolPtr))
  {
    if (Spinnaker::GenApi::IsWritable(boolPtr))
    {
      boolPtr->SetValue(value);
      ROS_INFO_STREAM("[SpinnakerCamera]: ("
                      << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") "
                      << property_name << " set to " << boolPtr->GetValue() << ".");
      return true;
    }
    else
    {
      ROS_WARN_STREAM("[SpinnakerCamera]: ("
                      << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue()
                      << ") Feature " << property_name << " not writable.");
    }
  }
  else
  {
    ROS_WARN_STREAM("[SpinnakerCamera]: ("
                    << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue()
                    << ") Feature " << property_name << " not available.");
  }
  return false;
}

inline bool setProperty(Spinnaker::GenApi::INodeMap* node_map, const std::string& property_name, const int& value)
{
  Spinnaker::GenApi::CIntegerPtr intPtr = node_map->GetNode(property_name.c_str());
  if (!Spinnaker::GenApi::IsImplemented(intPtr))
  {
    ROS_ERROR_STREAM("[SpinnakerCamera]: ("
                     << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue()
                     << ") Feature name " << property_name << " not implemented.");
    return false;
  }
  if (Spinnaker::GenApi::IsAvailable(intPtr))
  {
    if (Spinnaker::GenApi::IsWritable(intPtr))
    {
      int temp_value = value;
      if (temp_value > intPtr->GetMax())
        temp_value = intPtr->GetMax();
      else if (temp_value < intPtr->GetMin())
        temp_value = intPtr->GetMin();
      intPtr->SetValue(temp_value);
      ROS_INFO_STREAM("[SpinnakerCamera]: ("
                      << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") "
                      << property_name << " set to " << intPtr->GetValue() << ".");
      return true;
    }
    else
    {
      ROS_WARN_STREAM("[SpinnakerCamera]: ("
                      << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue()
                      << ") Feature " << property_name << " not writable.");
    }
  }
  else
  {
    ROS_WARN_STREAM("[SpinnakerCamera]: ("
                    << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue()
                    << ") Feature " << property_name << " not available.");
  }
  return false;
}

inline bool setMaxInt(Spinnaker::GenApi::INodeMap* node_map, const std::string& property_name)
{
  Spinnaker::GenApi::CIntegerPtr intPtr = node_map->GetNode(property_name.c_str());

  if (Spinnaker::GenApi::IsAvailable(intPtr))
  {
    if (Spinnaker::GenApi::IsWritable(intPtr))
    {
      intPtr->SetValue(intPtr->GetMax());
      ROS_INFO_STREAM("[SpinnakerCamera]: ("
                      << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") "
                      << property_name << " set to " << intPtr->GetValue() << ".");
      return true;
    }
    else
    {
      ROS_WARN_STREAM("[SpinnakerCamera]: ("
                      << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue()
                      << ") Feature " << property_name << " not writable.");
    }
  }
  else
  {
    ROS_WARN_STREAM("[SpinnakerCamera]: ("
                    << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue()
                    << ") Feature " << property_name << " not available.");
  }
  return false;
}
}  // namespace spinnaker_camera_driver
#endif  // SPINNAKER_CAMERA_DRIVER_SET_PROPERTY_H
