#ifndef SET_PROPERTY_H
#define SET_PROPERTY_H

// Spinnaker SDK
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

namespace flir_camera_driver
{
inline bool setProperty(Spinnaker::GenApi::INodeMap *node_map, const std::string& property_name, const std::string& entry_name)
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
    ROS_ERROR_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") Enumeration name " << property_name << " not "
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

          ROS_INFO_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") " << property_name << " set to "
                                                 << enumerationPtr->GetCurrentEntry()->GetSymbolic() << ".");

          return true;
        }
        else
        {
          ROS_WARN_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") Entry name " << entry_name << " not writable.");
        }
      }
      else
      {
        ROS_WARN_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") Entry name " << entry_name << " not available.");
      }
    }
    else
    {
      ROS_WARN_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") Enumeration " << property_name << " not writable.");
    }
  }
  else
  {
    ROS_WARN_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") Enumeration " << property_name << " not available.");
  }
  return false;
}

inline bool setProperty(Spinnaker::GenApi::INodeMap *node_map, const std::string& property_name, const float& value)
{
  Spinnaker::GenApi::CFloatPtr floatPtr = node_map->GetNode(property_name.c_str());

  if (!Spinnaker::GenApi::IsImplemented(floatPtr))
  {
    ROS_ERROR_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") Feature name " << property_name << " not implemented.");
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
      ROS_INFO_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") " << property_name << " set to " << floatPtr->GetValue()
                                             << ".");
      return true;
    }
    else
    {
      ROS_WARN_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") Feature " << property_name << " not writable.");
    }
  }
  else
  {
    ROS_WARN_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") Feature " << property_name << " not available.");
  }
  return false;
}

inline bool setProperty(Spinnaker::GenApi::INodeMap *node_map, const std::string& property_name, const bool& value)
{
  Spinnaker::GenApi::CBooleanPtr boolPtr = node_map->GetNode(property_name.c_str());
  if (!Spinnaker::GenApi::IsImplemented(boolPtr))
  {
    ROS_ERROR_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") Feature name " << property_name << " not implemented.");
    return false;
  }
  if (Spinnaker::GenApi::IsAvailable(boolPtr))
  {
    if (Spinnaker::GenApi::IsWritable(boolPtr))
    {
      boolPtr->SetValue(value);
      ROS_INFO_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") " << property_name << " set to " << boolPtr->GetValue()
                                             << ".");
      return true;
    }
    else
    {
      ROS_WARN_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") Feature " << property_name << " not writable.");
    }
  }
  else
  {
    ROS_WARN_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") Feature " << property_name << " not available.");
  }
  return false;
}

inline bool setProperty(Spinnaker::GenApi::INodeMap *node_map, const std::string& property_name, const int& value)
{
  Spinnaker::GenApi::CIntegerPtr intPtr = node_map->GetNode(property_name.c_str());
  if (!Spinnaker::GenApi::IsImplemented(intPtr))
  {
    ROS_ERROR_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") Feature name " << property_name << " not implemented.");
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
      ROS_INFO_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") " << property_name << " set to " << intPtr->GetValue()
                                             << ".");
      return true;
    }
    else
    {
      ROS_WARN_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") Feature " << property_name << " not writable.");
    }
  }
  else
  {
    ROS_WARN_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") Feature " << property_name << " not available.");
  }
  return false;
}

inline bool setMaxInt(Spinnaker::GenApi::INodeMap *node_map, const std::string& property_name)
{
  Spinnaker::GenApi::CIntegerPtr intPtr = node_map->GetNode(property_name.c_str());

  if (Spinnaker::GenApi::IsAvailable(intPtr))
  {
    if (Spinnaker::GenApi::IsWritable(intPtr))
    {
      intPtr->SetValue(intPtr->GetMax());
      ROS_INFO_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") " << property_name << " set to " << intPtr->GetValue()
                                             << ".");
      return true;
    }
    else
    {
      ROS_WARN_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") Feature " << property_name << " not writable.");
    }
  }
  else
  {
    ROS_WARN_STREAM("[FlirCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(node_map->GetNode("DeviceID"))->GetValue() << ") Feature " << property_name << " not available.");
  }
  return false;
}
}

#endif  // SET_PROPERTY_H
