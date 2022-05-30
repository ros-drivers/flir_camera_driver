/*
This code was developed by the National Robotics Engineering Center (NREC), part of the Robotics Institute at Carnegie
Mellon University.
Its development was funded by DARPA under the LS3 program and submitted for public release on June 7th, 2012.
Release was granted on August, 21st 2012 with Distribution Statement "A" (Approved for Public Release, Distribution
Unlimited).

This software is released under a BSD license:

Copyright (c) 2012, Carnegie Mellon University. All rights reserved.
Copyright (c) 2018, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.
Neither the name of the Carnegie Mellon University nor the names of its contributors may be used to endorse or promote
products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*-*-C++-*-*/
/**
   @file SpinnakerCamera.cpp
   @author Chad Rockey
   @date July 11, 2011
   @brief Interface to Point Grey cameras

   @attention Copyright (C) 2011
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

#include "spinnaker_camera_driver/SpinnakerCamera.h"

#include <iostream>
#include <sstream>
#include <typeinfo>
#include <string>

#include <ros/ros.h>

namespace spinnaker_camera_driver
{
SpinnakerCamera::SpinnakerCamera()
  : serial_(0)
  , system_(Spinnaker::System::GetInstance())
  , camList_(system_->GetCameras())
  , pCam_(static_cast<int>(NULL))  // Hack to suppress compiler warning. Spinnaker has only one contructor which takes
                                   // an int
  , camera_(static_cast<int>(NULL))
  , captureRunning_(false)
{
  unsigned int num_cameras = camList_.GetSize();
  ROS_INFO_STREAM_ONCE("[SpinnakerCamera]: Number of cameras detected: " << num_cameras);
}

SpinnakerCamera::~SpinnakerCamera()
{
  // @note ebretl Destructors of camList_ and system_ handle teardown
}

void SpinnakerCamera::setNewConfiguration(const spinnaker_camera_driver::SpinnakerConfig& config, const uint32_t& level)
{
  // Check if camera is connected
  if (!pCam_)
  {
    SpinnakerCamera::connect();
  }

  // Activate mutex to prevent us from grabbing images during this time
  std::lock_guard<std::mutex> scopedLock(mutex_);

  if (level >= LEVEL_RECONFIGURE_STOP)
  {
    ROS_DEBUG("SpinnakerCamera::setNewConfiguration: Reconfigure Stop.");
    bool capture_was_running = captureRunning_;
    start();  // For some reason some params only work after aquisition has be started once.
    stop();
    camera_->setNewConfiguration(config, level);
    if (capture_was_running)
      start();
  }
  else
  {
    camera_->setNewConfiguration(config, level);
  }
}  // end setNewConfiguration

void SpinnakerCamera::setGain(const float& gain)
{
  if (camera_)
    camera_->setGain(gain);
}

int SpinnakerCamera::getHeightMax()
{
  if (camera_)
    return camera_->getHeightMax();
  else
    return 0;
}

int SpinnakerCamera::getWidthMax()
{
  if (camera_)
    return camera_->getWidthMax();
  else
    return 0;
}

bool SpinnakerCamera::readableProperty(const Spinnaker::GenICam::gcstring property_name)
{
  if (camera_)
  {
    return camera_->readableProperty(property_name);
  }
  else
  {
    return 0;
  }
}

Spinnaker::GenApi::CNodePtr SpinnakerCamera::readProperty(const Spinnaker::GenICam::gcstring property_name)
{
  if (camera_)
  {
    return camera_->readProperty(property_name);
  }
  else
  {
    return 0;
  }
}

void SpinnakerCamera::findCameraPtr()
{
  // If we have a specific camera to connect to (specified by a serial number)
  if (serial_ != 0)
  {
    const auto serial_string = std::to_string(serial_);

    try
    {
      pCam_ = camList_.GetBySerial(serial_string);
    }
    catch (const Spinnaker::Exception& e)
    {
      throw std::runtime_error("[SpinnakerCamera::connect] Could not find camera with serial number " +
                                serial_string + ". Is that camera plugged in? Error: " + std::string(e.what()));
    }
  }
  else
  {
    // Connect to any camera (the first)
    try
    {
      pCam_ = camList_.GetByIndex(0);
    }
    catch (const Spinnaker::Exception& e)
    {
      throw std::runtime_error("[SpinnakerCamera::connect] Failed to get first connected camera. Error: " +
                                std::string(e.what()));
    }
  }
}

void SpinnakerCamera::updateCameraPtr()
{
  if (pCam_->IsInitialized())
  {
    pCam_->DeInit();
  }
  int prev_cam_list_size = camList_.GetSize();
  camList_.Clear();

  // If a parameter changed on a camera (e.g. IP address), we need to update the cameras
  for (size_t i = 0; i < 10; i++)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Delay to let time to completely release the camera
    system_->UpdateCameras();
    camList_ = system_->GetCameras();
    if (camList_.GetSize() == prev_cam_list_size)
    {
      break;
    }
  }
  if (camList_.GetSize() != prev_cam_list_size)
  {
    throw std::runtime_error("[SpinnakerCamera::updateCameraPtr] Failed to reconnect to camera after changing IP. "
                              "Try to reboot your camera.");
  }

  // For debugging purpose
  // for (size_t idx = 0; idx < camList_.GetSize(); idx++)
  // {
  //   Spinnaker::CameraPtr cam = camList_.GetByIndex(idx);
  //   Spinnaker::GenApi::INodeMap& node_map_local = cam->GetTLDeviceNodeMap();

  //   Spinnaker::GenApi::CStringPtr sn = node_map_local.GetNode("DeviceSerialNumber");
  //   std::cout << "Found SN " << sn->GetValue() << std::endl;
  // }

  // std::cout << "Targetting serial " << serial_ << std::endl;
  pCam_ = nullptr;
  findCameraPtr();
}

void SpinnakerCamera::connect()
{
  if (!pCam_)
  {
    findCameraPtr();
    if (!pCam_ || !pCam_->IsValid())
    {
      throw std::runtime_error("[SpinnakerCamera::connect] Failed to obtain camera reference.");
    }

    try
    {
      // Check Device type and save serial for reconnecting
      Spinnaker::GenApi::INodeMap& genTLNodeMap = pCam_->GetTLDeviceNodeMap();

      if (serial_ == 0)
      {
        Spinnaker::GenApi::CStringPtr serial_ptr =
            static_cast<Spinnaker::GenApi::CStringPtr>(genTLNodeMap.GetNode("DeviceSerialNumber"));
        if (IsAvailable(serial_ptr) && IsReadable(serial_ptr))
        {
          serial_ = atoi(serial_ptr->GetValue().c_str());
          ROS_INFO("[SpinnakerCamera::connect]: Using Serial: %i", serial_);
        }
        else
        {
          throw std::runtime_error("[SpinnakerCamera::connect]: Unable to determine serial number.");
        }
      }

      Spinnaker::GenApi::CEnumerationPtr device_type_ptr =
          static_cast<Spinnaker::GenApi::CEnumerationPtr>(genTLNodeMap.GetNode("DeviceType"));

      if (IsAvailable(device_type_ptr) && IsReadable(device_type_ptr))
      {
        ROS_INFO_STREAM("[SpinnakerCamera::connect]: Detected device type: " << device_type_ptr->ToString());

        if (device_type_ptr->GetCurrentEntry() == device_type_ptr->GetEntryByName("U3V"))
        {
          Spinnaker::GenApi::CEnumerationPtr device_speed_ptr =
              static_cast<Spinnaker::GenApi::CEnumerationPtr>(genTLNodeMap.GetNode("DeviceCurrentSpeed"));
          if (IsAvailable(device_speed_ptr) && IsReadable(device_speed_ptr))
          {
            if (device_speed_ptr->GetCurrentEntry() != device_speed_ptr->GetEntryByName("SuperSpeed"))
              ROS_ERROR_STREAM("[SpinnakerCamera::connect]: U3V Device not running at Super-Speed. Check Cables! ");
          }
        }
        // TODO(mhosmar): - check if interface is GigE and connect to GigE cam
        // Actually: if GigE camera, auto force IP if no IP is set in config 
        //   and if the camera is not on the right subnetwork,
        // or update IP according to config (if specified)
        if (device_type_ptr->GetIntValue() == Spinnaker::DeviceTypeEnum::DeviceType_GigEVision)
        {

          Spinnaker::GenApi::CBooleanPtr is_wrong_subnet = genTLNodeMap.GetNode("GevDeviceIsWrongSubnet");

          if ((ip_ == 0) && is_wrong_subnet->GetValue())
          {
            ROS_WARN_STREAM("NO IP set for camera with serial " << serial_ << ". "
                            << "Trying to auto force IP, which can lead to adresses issues.");
            tryAutoForceIP();
          }
          else if (ip_ != 0)
          {
            setIP();
          }

          if (pCam_->IsInitialized())
          {
            pCam_->DeInit();
          }
        }
      }
    }
    catch (const Spinnaker::Exception& e)
    {
      throw std::runtime_error("[SpinnakerCamera::connect] Failed to determine device info with error: " +
                               std::string(e.what()));
    }

    try
    {
      // Initialize Camera
      pCam_->Init();

      // Retrieve GenICam nodemap
      node_map_ = &pCam_->GetNodeMap();

      // detect model and set camera_ accordingly;
      Spinnaker::GenApi::CStringPtr model_name = node_map_->GetNode("DeviceModelName");
      std::string model_name_str(model_name->ToString());

      ROS_INFO("[SpinnakerCamera::connect]: Camera model name: %s", model_name_str.c_str());
      if (model_name_str.find("Blackfly S") != std::string::npos)
        camera_.reset(new Camera(node_map_));
      else if (model_name_str.find("Chameleon3") != std::string::npos)
        camera_.reset(new Cm3(node_map_));
      else if (model_name_str.find("Grasshopper3") != std::string::npos)
        camera_.reset(new Gh3(node_map_));
      else
      {
        camera_.reset(new Camera(node_map_));
        ROS_WARN("SpinnakerCamera::connect: Could not detect camera model name.");
      }

      // Need to do this here: no reference to camera in Camera class
      unsigned int packet_size_max = pCam_->DiscoverMaxPacketSize();
      camera_->setPacketSizeMax(packet_size_max);

      // Configure chunk data - Enable Metadata
      // SpinnakerCamera::ConfigureChunkData(*node_map_);
    }
    catch (const Spinnaker::Exception& e)
    {
      throw std::runtime_error("[SpinnakerCamera::connect] Failed to connect to camera. Error: " +
                               std::string(e.what()));
    }
    catch (const std::runtime_error& e)
    {
      throw std::runtime_error("[SpinnakerCamera::connect] Failed to configure chunk data. Error: " +
                               std::string(e.what()));
    }
  }

  // TODO(mhosmar): Get camera info to check if camera is running in color or mono mode
  /*
  CameraInfo cInfo;
  error = cam_.GetCameraInfo(&cInfo);
  SpinnakerCamera::handleError("SpinnakerCamera::connect  Failed to get camera info.", error);
  isColor_ = cInfo.isColorCamera;
  */
}

void SpinnakerCamera::disconnect()
{
  std::lock_guard<std::mutex> scopedLock(mutex_);
  captureRunning_ = false;
  try
  {
    // Check if camera is connected
    if (pCam_)
    {
      pCam_->DeInit();
      pCam_ = static_cast<int>(NULL);
      camList_.RemoveBySerial(std::to_string(serial_));
    }
    Spinnaker::CameraList temp_list = system_->GetCameras();
    camList_.Append(temp_list);
  }
  catch (const Spinnaker::Exception& e)
  {
    throw std::runtime_error("[SpinnakerCamera::disconnect] Failed to disconnect camera with error: " +
                             std::string(e.what()));
  }
}

void SpinnakerCamera::start()
{
  try
  {
    // Check if camera is connected
    if (pCam_ && !captureRunning_)
    {
      // Start capturing images
      pCam_->BeginAcquisition();
      captureRunning_ = true;
    }
  }
  catch (const Spinnaker::Exception& e)
  {
    throw std::runtime_error("[SpinnakerCamera::start] Failed to start capture with error: " + std::string(e.what()));
  }
}

void SpinnakerCamera::stop()
{
  if (pCam_ && captureRunning_)
  {
    // Stop capturing images
    try
    {
      captureRunning_ = false;
      pCam_->EndAcquisition();
    }
    catch (const Spinnaker::Exception& e)
    {
      throw std::runtime_error("[SpinnakerCamera::stop] Failed to stop capture with error: " + std::string(e.what()));
    }
  }
}

void SpinnakerCamera::grabImage(sensor_msgs::Image* image, const std::string& frame_id)
{
  std::lock_guard<std::mutex> scopedLock(mutex_);

  // Check if Camera is connected and Running
  if (pCam_ && captureRunning_)
  {
    // Handle "Image Retrieval" Exception
    try
    {
      Spinnaker::ImagePtr image_ptr = pCam_->GetNextImage(timeout_);
      //  std::string format(image_ptr->GetPixelFormatName());
      //  std::printf("\033[100m format: %s \n", format.c_str());

      //  throw std::runtime_error("[SpinnakerCamera::grabImage] Image received from camera "
      //                            + std::to_string(serial_)
      //                            + " is incomplete.");
      while (image_ptr->IsIncomplete())
      {
        ROS_WARN_STREAM_ONCE("[SpinnakerCamera::grabImage] Image received from camera "
                              << std::to_string(serial_)
                              << " is incomplete. Trying again.");
        image_ptr = pCam_->GetNextImage(timeout_);
      }

      // Set Image Time Stamp
      image->header.stamp.sec = image_ptr->GetTimeStamp() * 1e-9;
      image->header.stamp.nsec = image_ptr->GetTimeStamp();

      // Check the bits per pixel.
      size_t bitsPerPixel = image_ptr->GetBitsPerPixel();

      // --------------------------------------------------
      // Set the image encoding
      std::string imageEncoding = sensor_msgs::image_encodings::MONO8;

      Spinnaker::GenApi::CEnumerationPtr color_filter_ptr =
          static_cast<Spinnaker::GenApi::CEnumerationPtr>(node_map_->GetNode("PixelColorFilter"));

      Spinnaker::GenICam::gcstring color_filter_str = color_filter_ptr->ToString();
      Spinnaker::GenICam::gcstring bayer_rg_str = "BayerRG";
      Spinnaker::GenICam::gcstring bayer_gr_str = "BayerGR";
      Spinnaker::GenICam::gcstring bayer_gb_str = "BayerGB";
      Spinnaker::GenICam::gcstring bayer_bg_str = "BayerBG";

      // if(isColor_ && bayer_format != NONE)
      if (color_filter_ptr->GetCurrentEntry() != color_filter_ptr->GetEntryByName("None"))
      {
        if (bitsPerPixel == 16)
        {
          // 16 Bits per Pixel
          if (color_filter_str.compare(bayer_rg_str) == 0)
          {
            imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB16;
          }
          else if (color_filter_str.compare(bayer_gr_str) == 0)
          {
            imageEncoding = sensor_msgs::image_encodings::BAYER_GRBG16;
          }
          else if (color_filter_str.compare(bayer_gb_str) == 0)
          {
            imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG16;
          }
          else if (color_filter_str.compare(bayer_bg_str) == 0)
          {
            imageEncoding = sensor_msgs::image_encodings::BAYER_BGGR16;
          }
          else
          {
            throw std::runtime_error("[SpinnakerCamera::grabImage] Bayer format not recognized for 16-bit format.");
          }
        }
        else
        {
          // 8 Bits per Pixel
          if (color_filter_str.compare(bayer_rg_str) == 0)
          {
            imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB8;
          }
          else if (color_filter_str.compare(bayer_gr_str) == 0)
          {
            imageEncoding = sensor_msgs::image_encodings::BAYER_GRBG8;
          }
          else if (color_filter_str.compare(bayer_gb_str) == 0)
          {
            imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG8;
          }
          else if (color_filter_str.compare(bayer_bg_str) == 0)
          {
            imageEncoding = sensor_msgs::image_encodings::BAYER_BGGR8;
          }
          else
          {
            throw std::runtime_error("[SpinnakerCamera::grabImage] Bayer format not recognized for 8-bit format.");
          }
        }
      }
      else  // Mono camera or in pixel binned mode.
      {
        if (bitsPerPixel == 16)
        {
          imageEncoding = sensor_msgs::image_encodings::MONO16;
        }
        else if (bitsPerPixel == 24)
        {
          imageEncoding = sensor_msgs::image_encodings::RGB8;
        }
        else
        {
          imageEncoding = sensor_msgs::image_encodings::MONO8;
        }
      }

      int width = image_ptr->GetWidth();
      int height = image_ptr->GetHeight();
      int stride = image_ptr->GetStride();

      // ROS_INFO_ONCE("\033[93m wxh: (%d, %d), stride: %d \n", width, height, stride);
      fillImage(*image, imageEncoding, height, width, stride, image_ptr->GetData());
      image->header.frame_id = frame_id;
    }
    catch (const Spinnaker::Exception& e)
    {
      throw std::runtime_error("[SpinnakerCamera::grabImage] Failed to retrieve buffer with error: " +
                               std::string(e.what()));
    }
  }
  else if (pCam_)
  {
    throw CameraNotRunningException("[SpinnakerCamera::grabImage] Camera is currently not running.  Please start "
                                    "capturing frames first.");
  }
  else
  {
    throw std::runtime_error("[SpinnakerCamera::grabImage] Not connected to the camera.");
  }
}  // end grabImage

void SpinnakerCamera::setTimeout(const double& timeout)
{
  timeout_ = static_cast<uint64_t>(std::round(timeout * 1000));
}
void SpinnakerCamera::setDesiredCamera(const uint32_t& id)
{
  serial_ = id;
}
void SpinnakerCamera::setDesiredIP(const std::string& ip)
{
  ip_ = fromStrToIP(ip);
}
void SpinnakerCamera::setDesiredSubnetMask(const std::string& mask)
{
  mask_ = fromStrToIP(mask);
}
void SpinnakerCamera::setDesiredGateway(const std::string& gateway)
{
  gateway_ = fromStrToIP(gateway);
}

int64_t SpinnakerCamera::fromStrToIP(const std::string ip_str)
{
  int64_t ip_int = 0;
  std::ostringstream ip_part;
  const char *ip_char = ip_str.c_str();
  int n=3;
  for (size_t i=0 ; i<ip_str.size() ; i++)
  {
    if (*ip_char != '.')
    {
      ip_part << *ip_char;
      ip_char++;
    }
    else
    {
      ip_int += (atoi(ip_part.str().c_str()) * pow(256, n));

      ip_part = std::ostringstream();
      ip_char++;
      n--;
    }
  }
  ip_int += (atoi(ip_part.str().c_str()) * pow(256, n));

  return ip_int;
}

std::string SpinnakerCamera::convertIPtoStr(const int64_t ip_int)
{
  unsigned int ip_us = static_cast<unsigned int>(ip_int);
  std::ostringstream ip_str;
  ip_str << ((ip_us & 0xFF000000) >> 24) << ".";
  ip_str << ((ip_us & 0x00FF0000) >> 16) << ".";
  ip_str << ((ip_us & 0x0000FF00) >> 8) << ".";
  ip_str << (ip_us & 0x000000FF);

  return ip_str.str();
}

void SpinnakerCamera::setIP()
{
  try
  {
    // Need to deinit the camera before updating its IP
    bool is_cam_init = pCam_->IsInitialized();
    if (is_cam_init)
    {
      pCam_->DeInit();
    }

    if (gateway_ == 0)
    {
      gateway_ = (ip_ & mask_) + 1;
    }

    if (pCam_ != nullptr)
    {
      Spinnaker::GenApi::INodeMap& node_mapTL = pCam_->GetTLDeviceNodeMap();
      Spinnaker::GenApi::CEnumerationPtr device_type = node_mapTL.GetNode("DeviceType");
      if (device_type->GetIntValue() != Spinnaker::DeviceTypeEnum::DeviceType_GigEVision)
      {
        ROS_INFO("[SpinnakerCamera::setIP] IP can only be set for GigEVision system");
        return;
      }

      Spinnaker::GenApi::CStringPtr cam_serial = node_mapTL.GetNode("DeviceSerialNumber");

      Spinnaker::GenApi::CIntegerPtr curr_ip = node_mapTL.GetNode("GevDeviceIPAddress");
      if (ip_ == 0)
      {
        ROS_INFO_STREAM("[SpinnakerCamera::setIP] No changes made to camera's IP. Current value is "
                        << curr_ip->GetValue());
        return;
      }
      if (!IsReadable(curr_ip))
      {
        ROS_ERROR("[SpinnakerCamera::setIP] Cannot read the current IP address");
      }
      if (curr_ip->GetValue() == ip_)
      {
        return;
      }

      // First, update the network config at the TL level
      Spinnaker::GenApi::CIntegerPtr camTLIP = node_mapTL.GetNode("GevDeviceForceIPAddress");
      if (!IsWritable(camTLIP))
      {
        ROS_ERROR("[SpinnakerCamera::setIP] Cannot force IP on this device (TL level)");
        return;
      }
      else
      {
        camTLIP->SetValue(ip_);
        ROS_INFO_STREAM("[SpinnakerCamera::setIP] IP address set to " << convertIPtoStr(camTLIP->GetValue()));
      }

      Spinnaker::GenApi::CIntegerPtr camTL_mask = node_mapTL.GetNode("GevDeviceForceSubnetMask");

      if (!IsWritable(camTL_mask))
      {
        ROS_ERROR("[SpinnakerCamera::setIP] Cannot force subnet mask on this device (TL level)");
        return;
      }
      else
      {
        camTL_mask->SetValue(mask_);
        ROS_INFO_STREAM("[SpinnakerCamera::setIP] Subnet mask set to " << convertIPtoStr(camTL_mask->GetValue()));
      }

      Spinnaker::GenApi::CIntegerPtr camTL_gateway = node_mapTL.GetNode("GevDeviceForceGateway");
      if (!IsWritable(camTL_gateway))
      {
        ROS_ERROR("[SpinnakerCamera::setIP] Cannot force gateway on this device (TL level)");
        return;
      }
      else
      {
        camTL_gateway->SetValue(gateway_);
        ROS_INFO_STREAM("[SpinnakerCamera::setIP] Gateway set to " << convertIPtoStr(camTL_gateway->GetValue()));
      }

      Spinnaker::GenApi::CCommandPtr force_IP = node_mapTL.GetNode("GevDeviceForceIP");
      if (!IsWritable(force_IP))
      {
        ROS_ERROR("[SpinnakerCamera::setIP] Cannot execute device force IP on this device");
        return;
      }
      else
      {
        force_IP->Execute();
      }

      updateCameraPtr();
    }

    // Update parameters at camera level
    if (pCam_ != nullptr)
    {
      Spinnaker::GenApi::INodeMap& node_mapTL = pCam_->GetTLDeviceNodeMap();  // Updated camera => update TL node map
      Spinnaker::GenApi::CBooleanPtr is_wrong_subnet = node_mapTL.GetNode("GevDeviceIsWrongSubnet");

      Spinnaker::GenApi::CStringPtr cam_serial = node_mapTL.GetNode("DeviceSerialNumber");

      Spinnaker::GenApi::CIntegerPtr cam_ip = node_mapTL.GetNode("GevDeviceIPAddress");

      if (!is_wrong_subnet->GetValue())
      {
        pCam_->Init();
        Spinnaker::GenApi::INodeMap& node_map = pCam_->GetNodeMap();

        Spinnaker::GenApi::CBooleanPtr is_IPConfig_persistent = node_map.GetNode("GevCurrentIPConfigurationPersistentIP");
        if (!IsWritable(is_IPConfig_persistent))
        {
          ROS_ERROR("[SpinnakerCamera::setIP] Cannot set persistent IP address");
          return;
        }
        else
        {
          is_IPConfig_persistent->SetValue(true);
          ROS_INFO_STREAM("[SpinnakerCamera::setIP] Persistent IP config set to " << is_IPConfig_persistent->GetValue());
        }

        Spinnaker::GenApi::CIntegerPtr camIP = node_map.GetNode("GevPersistentIPAddress");
        if (!IsWritable(camIP))
        {
          ROS_ERROR("[SpinnakerCamera::setIP] Cannot force IP on this device (camera level)");
          is_IPConfig_persistent->SetValue(false);
          return;
        }
        else
        {
          camIP->SetValue(ip_);
        }

        Spinnaker::GenApi::CIntegerPtr cam_mask = node_map.GetNode("GevPersistentSubnetMask");
        if (!IsWritable(cam_mask))
        {
          ROS_ERROR("[SpinnakerCamera::setIP] Cannot force subnet mask on this device (camera level)");
          is_IPConfig_persistent->SetValue(false);
          return;
        }
        else
        {
          cam_mask->SetValue(mask_);
        }

        Spinnaker::GenApi::CIntegerPtr cam_gateway = node_map.GetNode("GevPersistentDefaultGateway");
        if (!IsWritable(cam_gateway))
        {
          ROS_ERROR("[SpinnakerCamera::setIP] Cannot force gateway on this device (camera level)");
          is_IPConfig_persistent->SetValue(false);
          return;
        }
        else
        {
          cam_gateway->SetValue(gateway_);
        }

        ROS_INFO("[SpinnakerCamera::setIP] Finished camera network configuration");
      }
      else
      {
        throw std::runtime_error("[SpinnakerCamera::setIP] Camera is still on the wrong subnet. "
                                  "Check your network configuration.");
      }

      // Let pCam_ state unchanged after changing IP
      if (!is_cam_init && pCam_->IsInitialized())
      {
        pCam_->DeInit();
      }
      else if (is_cam_init && !pCam_->IsInitialized())
      {
        pCam_->Init();
      }
    }
  }
  catch(const Spinnaker::Exception& e)
  {
    std::runtime_error(e.what());
  }
}  // end setIP

void SpinnakerCamera::tryAutoForceIP()
{
  Spinnaker::InterfaceList interfaces = system_->GetInterfaces();
  for (unsigned int i = 0; i < interfaces.GetSize(); i++)
  {
    Spinnaker::InterfacePtr pInterface = interfaces.GetByIndex(i);
    pInterface->UpdateCameras();
    Spinnaker::GenApi::INodeMap& nodeMapInterface = pInterface->GetTLNodeMap();

    Spinnaker::GenApi::CEnumerationPtr ptrInterfaceType = nodeMapInterface.GetNode("InterfaceType");
    if (!IsAvailable(ptrInterfaceType) || !IsReadable(ptrInterfaceType))
    {
      ROS_ERROR_STREAM("[SpinnakerCamera::tryAutoForceIP] Unable to read InterfaceType for interface at index " << i);
      continue;
    }

    if (ptrInterfaceType->GetIntValue() != Spinnaker::InterfaceTypeEnum::InterfaceType_GigEVision)
    {
      // Only force IP on GEV interface
      continue;
    }

    Spinnaker::GenApi::CCommandPtr ptrAutoForceIP = nodeMapInterface.GetNode("GevDeviceAutoForceIP");
    if (IsAvailable(ptrAutoForceIP) && IsWritable(ptrAutoForceIP))
    {
      if (!IsWritable(pInterface->TLInterface.DeviceSelector.GetAccessMode()))
      {
        ROS_ERROR("[SpinnakerCamera::tryAutoForceIP] Unable to write to the DeviceSelector node while forcing IP");
      }
      else
      {
        Spinnaker::CameraList cam_list = pInterface->GetCameras();
        for (int i = 0; i < cam_list.GetSize(); i++)
        {
          Spinnaker::CameraPtr pCam = cam_list.GetByIndex(i);
          if (!pCam_->GetUniqueID().compare(pCam->GetUniqueID()))
          {
            Spinnaker::GenApi::CStringPtr cam_serial = pCam->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");
            ROS_INFO_STREAM("[SpinnakerCamera::tryAutoForceIP] Forced IP for camera with serial "
                            << cam_serial->GetValue());

            pInterface->TLInterface.DeviceSelector.SetValue(i);
            pInterface->TLInterface.GevDeviceAutoForceIP.Execute();
            break;
          }
        }
        break;
      }
    }
    else
    {
      ROS_WARN("[SpinnakerCamera::tryAutoForceIP] AutoForceIP not available for this interface");
    }
  }

  interfaces.Clear();
  updateCameraPtr();
}

void SpinnakerCamera::ConfigureChunkData(const Spinnaker::GenApi::INodeMap& nodeMap)
{
  ROS_INFO_STREAM("*** CONFIGURING CHUNK DATA ***");
  try
  {
    // Activate chunk mode
    //
    // *** NOTES ***
    // Once enabled, chunk data will be available at the end of the payload
    // of every image captured until it is disabled. Chunk data can also be
    // retrieved from the nodemap.
    //
    Spinnaker::GenApi::CBooleanPtr ptrChunkModeActive = nodeMap.GetNode("ChunkModeActive");
    if (!Spinnaker::GenApi::IsAvailable(ptrChunkModeActive) || !Spinnaker::GenApi::IsWritable(ptrChunkModeActive))
    {
      throw std::runtime_error("Unable to activate chunk mode. Aborting...");
    }
    ptrChunkModeActive->SetValue(true);
    ROS_INFO_STREAM_ONCE("Chunk mode activated...");

    // Enable all types of chunk data
    //
    // *** NOTES ***
    // Enabling chunk data requires working with nodes: "ChunkSelector"
    // is an enumeration selector node and "ChunkEnable" is a boolean. It
    // requires retrieving the selector node (which is of enumeration node
    // type), selecting the entry of the chunk data to be enabled, retrieving
    // the corresponding boolean, and setting it to true.
    //
    // In this example, all chunk data is enabled, so these steps are
    // performed in a loop. Once this is complete, chunk mode still needs to
    // be activated.
    //
    Spinnaker::GenApi::NodeList_t entries;
    // Retrieve the selector node
    Spinnaker::GenApi::CEnumerationPtr ptrChunkSelector = nodeMap.GetNode("ChunkSelector");
    if (!Spinnaker::GenApi::IsAvailable(ptrChunkSelector) || !Spinnaker::GenApi::IsReadable(ptrChunkSelector))
    {
      throw std::runtime_error("Unable to retrieve chunk selector. Aborting...");
    }
    // Retrieve entries
    ptrChunkSelector->GetEntries(entries);

    ROS_INFO_STREAM("Enabling entries...");

    for (unsigned int i = 0; i < entries.size(); i++)
    {
      // Select entry to be enabled
      Spinnaker::GenApi::CEnumEntryPtr ptrChunkSelectorEntry = entries.at(i);
      // Go to next node if problem occurs
      if (!Spinnaker::GenApi::IsAvailable(ptrChunkSelectorEntry) ||
          !Spinnaker::GenApi::IsReadable(ptrChunkSelectorEntry))
      {
        continue;
      }
      ptrChunkSelector->SetIntValue(ptrChunkSelectorEntry->GetValue());

      ROS_INFO_STREAM("\t" << ptrChunkSelectorEntry->GetSymbolic() << ": ");
      // Retrieve corresponding boolean
      Spinnaker::GenApi::CBooleanPtr ptrChunkEnable = nodeMap.GetNode("ChunkEnable");
      // Enable the boolean, thus enabling the corresponding chunk data
      if (!Spinnaker::GenApi::IsAvailable(ptrChunkEnable))
      {
        ROS_INFO("Node not available");
      }
      else if (ptrChunkEnable->GetValue())
      {
        ROS_INFO("Enabled");
      }
      else if (Spinnaker::GenApi::IsWritable(ptrChunkEnable))
      {
        ptrChunkEnable->SetValue(true);
        ROS_INFO("Enabled");
      }
      else
      {
        ROS_INFO("Node not writable");
      }
    }
  }
  catch (const Spinnaker::Exception& e)
  {
    throw std::runtime_error(e.what());
  }
}
}  // namespace spinnaker_camera_driver
