/*
This code was developed by the National Robotics Engineering Center (NREC), part of the Robotics Institute at Carnegie Mellon University.
Its development was funded by DARPA under the LS3 program and submitted for public release on June 7th, 2012.
Release was granted on August, 21st 2012 with Distribution Statement "A" (Approved for Public Release, Distribution Unlimited).

This software is released under a BSD license:

Copyright (c) 2012, Carnegie Mellon University. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
Neither the name of the Carnegie Mellon University nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/



/*-*-C++-*-*/
/**
   @file FlirCamera.cpp
   @author Chad Rockey
   @date July 11, 2011
   @brief Interface to Point Grey cameras

   @attention Copyright (C) 2011
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

#include "flir_camera_driver/FlirCamera.h"

#include <iostream>
#include <sstream>
#include <typeinfo>

#include <ros/ros.h>

namespace flir_camera_driver {
FlirCamera::FlirCamera()
  : serial_(0)
  , system_(Spinnaker::System::GetInstance())
  , camList_(system_->GetCameras())
  , pCam_(NULL)
  , camera_(NULL)
  , captureRunning_(false)
{
  unsigned int num_cameras = camList_.GetSize();
  ROS_INFO_STREAM_ONCE("[FlirCamera]: Number of cameras detected: " <<  num_cameras);

}

FlirCamera::~FlirCamera()
{
  camList_.Clear();
  system_->ReleaseInstance();
}

bool FlirCamera::setNewConfiguration(flir_camera_driver::FlirConfig &config, const uint32_t &level)
{
  // Check if camera is connected
  if(!pCam_)
  {
    FlirCamera::connect();
  }

  bool ret_val = false;
  // Activate mutex to prevent us from grabbing images during this time
  std::lock_guard<std::mutex> scopedLock(mutex_);

  if(level >= LEVEL_RECONFIGURE_STOP)
  {
    ROS_DEBUG("FlirCamera::setNewConfiguration: Reconfigure Stop.");
    bool capture_was_running = captureRunning_;
    start(); // For some reason some params only work after aquisition has be started once.
    stop();
    ret_val = camera_->setNewConfiguration(config, level);
    if (capture_was_running)
      start();
  }
  else
  {
    ret_val = camera_->setNewConfiguration(config, level);
  }

  return ret_val;
}  // end setNewConfiguration


void FlirCamera::setGain(const float& gain)
{
  if (camera_)
    camera_->setGain(gain);
}

uint FlirCamera::getHeightMax()
{
  if(camera_)
    return camera_->getHeightMax();
  else
    return 0;
}

uint FlirCamera::getWidthMax()
{
  if (camera_)
    return camera_->getWidthMax();
  else
    return 0;
}

int FlirCamera::connect()
{
  int result = 0;
  int err = 0;

  if(!pCam_)
  {
    // If we have a specific camera to connect to (specified by a serial number)
    if(serial_ != 0)
    {
      const auto serial_string = std::to_string(serial_);

      try
      {
        pCam_ = camList_.GetBySerial(serial_string);
      }
      catch (const Spinnaker::Exception &e)
      {
        ROS_ERROR_STREAM_ONCE("FlirCamera::connect Could not find camera with serial number: %s Is that camera plugged in? Error: " << e.what());
        result = -1;
      }
    }
    else
    {
      // Connect to any camera (the first)

      try
      {
        pCam_ = camList_.GetByIndex(0);
      }
      catch (const Spinnaker::Exception &e)
      {
        ROS_ERROR_STREAM_ONCE("FlirCamera::connect Failed to get first connected camera. Error: " << e.what());
        result = -1;
      }
    }

    Spinnaker::GenApi::INodeMap & genTLNodeMap = pCam_->GetTLDeviceNodeMap();
    Spinnaker::GenApi::CEnumerationPtr device_type_ptr =
                                    static_cast<Spinnaker::GenApi::CEnumerationPtr>(genTLNodeMap.GetNode("DeviceType"));

    if(IsAvailable(device_type_ptr) && IsReadable(device_type_ptr)){
      ROS_INFO_STREAM("FlirCamera::connect Detected device type: " << device_type_ptr->ToString());

      if(device_type_ptr->GetCurrentEntry() == device_type_ptr->GetEntryByName("U3V"))
      {
        Spinnaker::GenApi::CEnumerationPtr device_speed_ptr =
                            static_cast<Spinnaker::GenApi::CEnumerationPtr>(genTLNodeMap.GetNode("DeviceCurrentSpeed"));
        if(IsAvailable(device_speed_ptr) && IsReadable(device_speed_ptr))
        {
          if(device_speed_ptr->GetCurrentEntry() != device_speed_ptr->GetEntryByName("SuperSpeed"))
            ROS_ERROR_STREAM("FlirCamera::connect U3V Device not running at Super-Speed. Check Cables! ");
        }
      }
      // TODO @tthomas - check if interface is GigE and connect to GigE cam
    }

    try
    {
      // Initialize Camera
      pCam_->Init();

      // Retrieve GenICam nodemap
      node_map_ = &pCam_->GetNodeMap();

      //TODO @mhosmar - detect model and set camera_ accordingly;
      camera_.reset(new Camera(node_map_));

      // Configure chunk data - Enable Metadata
      // err = FlirCamera::ConfigureChunkData(*node_map_);
      if (err < 0)
      {
        return err;
      }
    }
    catch (const Spinnaker::Exception &e)
    {
      ROS_ERROR_STREAM_ONCE("FlirCamera::connect Failed to connect to camera. Error: " << e.what());
      result = -1;
    }

    // TODO: Get camera info to check if camera is running in color or mono mode
    /*
    CameraInfo cInfo;
    error = cam_.GetCameraInfo(&cInfo);
    FlirCamera::handleError("FlirCamera::connect  Failed to get camera info.", error);
    isColor_ = cInfo.isColorCamera;
    */
  }
  return result;
}

int FlirCamera::disconnect()
{
  int result = 0;

  std::lock_guard<std::mutex> scopedLock(mutex_);
  captureRunning_ = false;

  // Check if camera is connected
  if(pCam_)
  {
    try
    {
      pCam_->DeInit();
    }
    catch (const Spinnaker::Exception &e)
    {
      ROS_ERROR_STREAM_ONCE("FlirCamera::disconnect Failed to disconnect camera with Error: " << e.what());
      result = -1;
    }
  }

  return result;
}

int FlirCamera::start()
{
  int result = 0;

  try
  {
    // Check if camera is connected
    if(pCam_ && !captureRunning_)
    {
      // Start capturing images
      pCam_->BeginAcquisition();
      captureRunning_ = true;
    }
  }
  catch (Spinnaker::Exception &e)
  {
    ROS_ERROR_STREAM_ONCE("FlirCamera::start Failed to start capture with Error: " << e.what());
    result = -1;
  }
  return result;
}


bool FlirCamera::stop()
{
  if (pCam_ && captureRunning_)
  {
    // Stop capturing images
    try
    {
      captureRunning_ = false;
      pCam_->EndAcquisition();
      return true;
    }
    catch (const Spinnaker::Exception &e)
    {
      ROS_ERROR_STREAM_ONCE("FlirCamera::stop Failed to stop capture with Error: " << e.what());
      return false;
    }
  }
  return false;
}


int FlirCamera::grabImage(sensor_msgs::Image &image, const std::string &frame_id)
{
  int result = 0;

  std::lock_guard<std::mutex> scopedLock(mutex_);

  // Check if Camera is connected and Running
  if (pCam_ && captureRunning_)
  {
    // Handle "Image Retrieval" Exception
    try
    {
      Spinnaker::ImagePtr image_ptr = pCam_->GetNextImage();
      //  std::string format(image_ptr->GetPixelFormatName());
      //  std::printf("\033[100m format: %s \n", format.c_str());

      if (image_ptr->IsIncomplete())
      {
        ROS_ERROR_ONCE("Camera %d Received but is Incomplete", serial_);
      }
      else
      {
        // Set Image Time Stamp
        image.header.stamp.sec = image_ptr->GetTimeStamp() * 1e-9;
        image.header.stamp.nsec = image_ptr->GetTimeStamp();

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
              ROS_ERROR_ONCE("Bayer Format Not Recognized!");
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
              ROS_ERROR_ONCE("Bayer Format Not Recognized!");
            }
          }
        }
        else     // Mono camera or in pixel binned mode.
        {

          if(bitsPerPixel == 16)
          {
            imageEncoding = sensor_msgs::image_encodings::MONO16;
          }
          else if(bitsPerPixel==24)
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
        fillImage(image, imageEncoding, height, width, stride, image_ptr->GetData());
        image.header.frame_id = frame_id;

      }  // end else
    }
    catch (const Spinnaker::Exception& e)
    {
      ROS_ERROR_STREAM_ONCE("FlirCamera::grabImage Failed to retrieve buffer with Error: " << e.what());
      result = -1;
    }

  }
  else if(pCam_)
  {
    throw CameraNotRunningException("FlirCamera::grabImage: Camera is currently not running.  Please start the capture.");
  }
  else
  {
    throw std::runtime_error("FlirCamera::grabImage not connected!");
  }

  return result;

}  // end grabImage


// TODO: @tthomas - Implement later if needed
void FlirCamera::grabStereoImage(sensor_msgs::Image &image, const std::string &frame_id,
  sensor_msgs::Image &second_image, const std::string &second_frame_id)
{

}

//void Camera::setTimeout(const double &timeout)
//{
//}


void FlirCamera::setDesiredCamera(const uint32_t &id)
{
  serial_ = id;
}

// TODO(tthomas)
// std::vector<uint32_t> FlirCamera::getAttachedCameras()
// {
//   std::vector<uint32_t> cameras;
//   unsigned int num_cameras;
//   Error error = busMgr_.GetNumOfCameras(&num_cameras);
//   FlirCamera::handleError("FlirCamera::getAttachedCameras: Could not get number of cameras", error);
//   for(unsigned int i = 0; i < num_cameras; i++)
//   {
//     unsigned int this_serial;
//     error = busMgr_.GetCameraSerialNumberFromIndex(i, &this_serial);
//     FlirCamera::handleError("FlirCamera::getAttachedCameras: Could not get get serial number from index", error);
//     cameras.push_back(this_serial);
//   }
//   return cameras;
// }


int FlirCamera::ConfigureChunkData(Spinnaker::GenApi::INodeMap & nodeMap)
{
  int result = 0;
  ROS_INFO_STREAM_ONCE("*** CONFIGURING CHUNK DATA ***");
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
      ROS_ERROR_STREAM_ONCE("Unable to activate chunk mode. Aborting...");
      return -1;
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
      ROS_ERROR_STREAM_ONCE("Unable to retrieve chunk selector. Aborting...");
      return -1;
    }
    // Retrieve entries
    ptrChunkSelector->GetEntries(entries);

    ROS_INFO_STREAM_ONCE("Enabling entries...");

    for (unsigned int i = 0; i < entries.size(); i++)
    {
      // Select entry to be enabled
      Spinnaker::GenApi::CEnumEntryPtr ptrChunkSelectorEntry = entries.at(i);
      // Go to next node if problem occurs
      if (!Spinnaker::GenApi::IsAvailable(ptrChunkSelectorEntry) || !Spinnaker::GenApi::IsReadable(ptrChunkSelectorEntry))
      {
        continue;
      }
      ptrChunkSelector->SetIntValue(ptrChunkSelectorEntry->GetValue());

      ROS_INFO_STREAM_ONCE("\t" << ptrChunkSelectorEntry->GetSymbolic() << ": ");
      // Retrieve corresponding boolean
      Spinnaker::GenApi::CBooleanPtr ptrChunkEnable = nodeMap.GetNode("ChunkEnable");
      // Enable the boolean, thus enabling the corresponding chunk data
      if (!Spinnaker::GenApi::IsAvailable(ptrChunkEnable))
      {
        ROS_INFO_ONCE("Node not available");
      }
      else if (ptrChunkEnable->GetValue())
      {
        ROS_INFO_ONCE("Enabled");
      }
      else if (Spinnaker::GenApi::IsWritable(ptrChunkEnable))
      {
        ptrChunkEnable->SetValue(true);
        ROS_INFO_ONCE("Enabled");
      }
      else
      {
        ROS_INFO_ONCE("Node not writable");
      }
    }
  }
  catch (const Spinnaker::Exception &e)
  {
    ROS_ERROR_STREAM_ONCE("Error: " << e.what());
    result = -1;
  }
  return result;
}
}
