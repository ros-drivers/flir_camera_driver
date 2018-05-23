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
   @file PointGreyCamera.cpp
   @author Chad Rockey
   @date July 11, 2011
   @brief Interface to Point Grey cameras

   @attention Copyright (C) 2011
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

#include "pointgrey_camera_driver/PointGreyCamera.h"

#include <iostream>
#include <sstream>
#include <typeinfo>

#include <ros/ros.h>


PointGreyCamera::PointGreyCamera()
  : system_(Spinnaker::System::GetInstance())
  , camList_(system_->GetCameras())
  , pCam_(NULL)
  , serial_(0)
  , captureRunning_(false)
{
  unsigned int num_cameras = camList_.GetSize();
  ROS_INFO_STREAM_ONCE("[PointGreyCamera]: Number of cameras detected: " <<  num_cameras);

}

PointGreyCamera::~PointGreyCamera()
{
}


bool PointGreyCamera::setFrameRate(const float frame_rate)
{
  // This enables the "AcquisitionFrameRateEnabled"
  //======================================

  Spinnaker::GenApi::CBooleanPtr ptrAcquisitionFrameRateEnable = node_map_->GetNode("AcquisitionFrameRateEnabled");
  if (!IsAvailable(ptrAcquisitionFrameRateEnable) || !IsWritable(ptrAcquisitionFrameRateEnable))
  {
    ROS_ERROR_ONCE("Unable to enable the AcquisitionFrameRateEnable. Aborting... \n");
    return false;
  }
  ptrAcquisitionFrameRateEnable->SetValue(true);
  //=============================================================================


  // This sets the "AcquisitionFrameRateAuto" to "Off"
  //======================================

  Spinnaker::GenApi::CEnumerationPtr ptrFrameRateAuto = node_map_->GetNode("AcquisitionFrameRateAuto");
  if (!IsAvailable(ptrFrameRateAuto) || !IsWritable(ptrFrameRateAuto))
  {
    ROS_ERROR_ONCE("Unable to set FrameRateAuto to continuous (enum retrieval). Aborting...\n");
    return false;
  }

  Spinnaker::GenApi::CEnumEntryPtr ptrFrameRateAutoOff = ptrFrameRateAuto->GetEntryByName("Off");
  if (!IsAvailable(ptrFrameRateAutoOff) || !IsReadable(ptrFrameRateAutoOff))
  {
    ROS_ERROR_ONCE("Unable to set FrameRateAuto to continuous (enum retrieval). Aborting...\n");
    return false;
  }

  int64_t frameRateAutoOff = ptrFrameRateAutoOff->GetValue();
  ptrFrameRateAuto->SetIntValue(frameRateAutoOff);
  //=============================================================================


  // This sets the "AcquisitionFrameRate" to X FPS
  // ========================================

  Spinnaker::GenApi::CFloatPtr ptrAcquisitionFrameRate = node_map_->GetNode("AcquisitionFrameRate");
  if (!IsAvailable(ptrAcquisitionFrameRate) || !IsWritable(ptrAcquisitionFrameRate))
  {
    ROS_ERROR_ONCE("Unable to set AcquisitionFrameRate. Aborting...\n");
    return false;
  }

  ROS_DEBUG_STREAM_ONCE("Minimum Frame Rate: \t " << ptrAcquisitionFrameRate->GetMin());
  ROS_DEBUG_STREAM_ONCE("Maximum Frame rate: \t " << ptrAcquisitionFrameRate->GetMax());


  // Finally Set the Frame Rate
  ptrAcquisitionFrameRate->SetValue(frame_rate);


  ROS_DEBUG_STREAM_ONCE("Current Frame rate: \t " << ptrAcquisitionFrameRate->GetValue());
}

bool PointGreyCamera::setNewConfiguration(pointgrey_camera_driver::PointGreyConfig &config, const uint32_t &level)
{
  // Check if camera is connected
  if(!pCam_)
  {
    PointGreyCamera::connect();
  }

  // Activate mutex to prevent us from grabbing images during this time
  boost::mutex::scoped_lock scopedLock(mutex_);

  // return true if we can set values as desired.
  bool retVal = true;

  float temp_frame_rate = config.acquisition_frame_rate;
  retVal = setFrameRate(temp_frame_rate);

  // Set Trigger and Strobe Settings
  // NOTE: The trigger must be disabled (i.e. TriggerMode = "Off") in order to configure whether the source is software or hardware.
  retVal = setProperty("TriggerMode", std::string("Off"));
  retVal = setProperty("TriggerSource", config.trigger_source);
  retVal = setProperty("TriggerSelector", config.trigger_selector);
  retVal = setProperty("TriggerActivation", config.trigger_activation_mode);
  retVal = setProperty("TriggerMode", config.enable_trigger);

  retVal = setProperty("LineSelector", config.line_selector);
  retVal = setProperty("LineMode", config.line_mode);
  retVal = setProperty("LineSource", config.line_source);

  // Set auto exposure
  retVal = setProperty("ExposureMode", config.exposure_mode);
  retVal = setProperty("ExposureAuto", config.exposure_auto);


  // Set Video Mode, Image and Pixel formats
  // retVal = PointGreyCamera::setVideoMode(config.video_mode);
  // retVal = PointGreyCamera::setImageControlFormats(config);

  /*
  TODO @tthomas: Revisit/Debug setProperty method for setting frame rate and other properties
  retVal = setProperty("AcquisitionFrameRateAuto", "Off");
  retVal = setProperty("AcquisitionFrameRateEnabled", true);


  // retVal = setProperty("AcquisitionFrameRate", config.acquisition_frame_rate);
  // TODO @tthomas: streamline double& to float& conversions
  float temp_frame_rate = config.acquisition_frame_rate;
  retVal = setProperty("AcquisitionFrameRate", temp_frame_rate);  // Feature AcquisitionFrameRate not writable.
  */


  // Set sharpness
  if (config.sharpening_enable)
  {
    retVal = setProperty("SharpeningAuto", config.auto_sharpness);
    // retVal = setProperty("Sharpening", config.sharpness);
    float temp_sharpness = config.sharpness;
    float temp_sharpening_threshold = config.sharpening_threshold;
    retVal = setProperty("Sharpening", temp_sharpness);
    retVal = setProperty("SharpeningThreshold", temp_sharpening_threshold);
  }

  // Set saturation
  if (config.saturation_enable)
  {
    retVal = setProperty("SaturationEnable", config.saturation_enable);
    float temp_saturation = config.saturation;
    retVal = setProperty("Saturation", temp_saturation);
  }


  // Set shutter time/speed
  if (config.exposure_auto.compare(std::string("Off")) == 0)
  {
    float temp_exposure_time = config.exposure_time;
    retVal = setProperty("ExposureTime", temp_exposure_time);
  }

  float temp_auto_exposure_exposure_time_upper_limit= config.auto_exposure_time_upper_limit;
  retVal = setProperty("AutoExposureTimeUpperLimit", temp_auto_exposure_exposure_time_upper_limit);


  // Set gain
  retVal = setProperty("GainSelector", config.gain_selector);
  retVal = setProperty("GainAuto", config.auto_gain);

  float temp_gain = config.gain;
  retVal = setProperty("Gain", temp_gain);

  // Set brightness
  float temp_brightness = config.brightness;
  retVal = setProperty("BlackLevel", temp_brightness);

  // Set gamma
  if (config.gamma_enable)
  {
    retVal = setProperty("GammaEnable", config.gamma_enable);

    float temp_gamma = config.gamma;
    retVal = setProperty("Gamma", temp_gamma);
  }

  // Set white balance
  retVal = setProperty("BalanceWhiteAuto", config.auto_white_balance);
  retVal = setProperty("BalanceRatioSelector", "Blue");

  float temp_white_balance_blue_ratio = config.white_balance_blue_ratio;
  retVal = setProperty("BalanceRatio", temp_white_balance_blue_ratio);

  retVal = setProperty("BalanceRatioSelector", "Red");
  float temp_white_balance_red_ratio = config.white_balance_red_ratio;
  retVal = setProperty("BalanceRatio", temp_white_balance_red_ratio);



  return retVal;

}  // end setNewConfiguration


void PointGreyCamera::setGain(const float& gain)
{
  // Turn auto gain off
  pCam_->GainAuto.SetValue(Spinnaker::GainAutoEnums::GainAuto_Off);

  // Set gain
  setProperty("Gain", gain);
}



bool PointGreyCamera::setVideoMode(const std::string& videoMode)
{

  ROS_INFO_STREAM_ONCE("\n\n videoMode: " << videoMode << "\n\n");

  // return true if we can set the video mode as desired.
  bool retVal = true;

  if (videoMode.compare("1280x960") == 0)
  {
    retVal = setProperty("VideoMode", "Mode0");
  }
  else if (videoMode.compare("640x480_pixel_aggregation") == 0)
  {
    retVal = setProperty("VideoMode", "Mode1");
  }
  else if (videoMode.compare("640x480_pixel_decimation") == 0)
  {
    retVal = setProperty("VideoMode", "Mode4");
  }
  else if (videoMode.compare("320x240") == 0)
  {
    retVal = setProperty("VideoMode", "Mode5");
  }
  else
  {
    ROS_ERROR_ONCE("Video Mode Unknown!");
    retVal = false;
  }

  return retVal;

}

// Image Size and Pixel Format
bool PointGreyCamera::setImageControlFormats(pointgrey_camera_driver::PointGreyConfig &config)
{

  // return true if we can set values as desired.
  bool retVal = true;

  // Apply minimum to offset X
  retVal = setProperty("OffsetX", config.image_format_x_offset);
  // Apply minimum to offset Y
  retVal = setProperty("OffsetY", config.image_format_y_offset);

  // Set maximum width
  retVal = setProperty("Width", config.image_format_roi_width);
  retVal = setProperty("Height", config.image_format_roi_height);

  // Set Pixel Format
  retVal = setProperty("PixelFormat", config.image_format_color_coding);

  return retVal;
}





/*

void PointGreyCamera::setTimeout(const double &timeout)
{
}

float PointGreyCamera::getCameraTemperature()
{
}

float PointGreyCamera::getCameraFrameRate()
{
}


void PointGreyCamera::setGigEParameters(bool auto_packet_size, unsigned int packet_size, unsigned int packet_delay)
{
}

void PointGreyCamera::setupGigEPacketSize(PGRGuid & guid)
{
}

void PointGreyCamera::setupGigEPacketSize(PGRGuid & guid, unsigned int packet_size)
{

}

void PointGreyCamera::setupGigEPacketDelay(PGRGuid & guid, unsigned int packet_delay)
{
}

*/

int PointGreyCamera::connect()
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
        ROS_ERROR_STREAM_ONCE("PointGreyCamera::connect Could not find camera with serial number: %s Is that camera plugged in? Error: " << e.what());
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
        ROS_ERROR_STREAM_ONCE("PointGreyCamera::connect Failed to get first connected camera. Error: " << e.what());
        result = -1;
      }
    }

    // TODO @tthomas - check if interface is GigE and connect to GigE cam

    try
    {
      // Initialize Camera
      pCam_->Init();

      // Retrieve GenICam nodemap
      node_map_ = &pCam_->GetNodeMap();

      // Configure chunk data - Enable Metadata
      // err = PointGreyCamera::ConfigureChunkData(*node_map_);
      if (err < 0)
      {
        return err;
      }

      // NOTE: Brightness is termed black level in GenICam
      float black_level = 1.7;
      setProperty("BlackLevel", black_level);
    }
    catch (const Spinnaker::Exception &e)
    {
      ROS_ERROR_STREAM_ONCE("PointGreyCamera::connect Failed to connect to camera. Error: " << e.what());
      result = -1;
    }
    return result;

    // TODO: Get camera info to check if camera is running in color or mono mode
    /*
    CameraInfo cInfo;
    error = cam_.GetCameraInfo(&cInfo);
    PointGreyCamera::handleError("PointGreyCamera::connect  Failed to get camera info.", error);
    isColor_ = cInfo.isColorCamera;
    */
  }
}

int PointGreyCamera::disconnect()
{
  int result = 0;

  boost::mutex::scoped_lock scopedLock(mutex_);
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
      ROS_ERROR_STREAM_ONCE("PointGreyCamera::disconnect Failed to disconnect camera with Error: " << e.what());
      result = -1;
    }
  }

  return result;
}

int PointGreyCamera::start()
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
    ROS_ERROR_STREAM_ONCE("PointGreyCamera::start Failed to start capture with Error: " << e.what());
    int result = -1;
  }
  return result;
}


bool PointGreyCamera::stop()
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
      ROS_ERROR_STREAM_ONCE("PointGreyCamera::stop Failed to stop capture with Error: " << e.what());
      return false;
    }
  }
  return false;
}


int PointGreyCamera::grabImage(sensor_msgs::Image &image, const std::string &frame_id)
{
  int result = 0;

  boost::mutex::scoped_lock scopedLock(mutex_);

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

        Spinnaker::GenApi::IEnumerationT<Spinnaker::PixelColorFilterEnums>& bayer_format = pCam_->PixelColorFilter;


        Spinnaker::GenICam::gcstring bayer_rg_str = "BayerRG";
        Spinnaker::GenICam::gcstring bayer_gr_str = "BayerGR";
        Spinnaker::GenICam::gcstring bayer_gb_str = "BayerGB";
        Spinnaker::GenICam::gcstring bayer_bg_str = "BayerBG";

        // if(isColor_ && bayer_format != NONE)
        if (bayer_format != Spinnaker::PixelColorFilter_None)
        {
          if (bitsPerPixel == 16)
          {
            // 16 Bits per Pixel
            if ((*bayer_format).compare(bayer_rg_str) == 0)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB16;
            }
            else if ((*bayer_format).compare(bayer_gr_str) == 0)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_GRBG16;
            }
            else if ((*bayer_format).compare(bayer_gb_str) == 0)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG16;
            }
            else if ((*bayer_format).compare(bayer_bg_str) == 0)
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
            if ((*bayer_format).compare(bayer_rg_str) == 0)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB8;
            }
            else if ((*bayer_format).compare(bayer_gr_str) == 0)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_GRBG8;
            }
            else if ((*bayer_format).compare(bayer_gb_str) == 0)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG8;
            }
            else if ((*bayer_format).compare(bayer_bg_str) == 0)
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
      ROS_ERROR_STREAM_ONCE("PointGreyCamera::grabImage Failed to retrieve buffer with Error: " << e.what());
      result = -1;
    }

  }
  else if(pCam_)
  {
    throw CameraNotRunningException("PointGreyCamera::grabImage: Camera is currently not running.  Please start the capture.");
  }
  else
  {
    throw std::runtime_error("PointGreyCamera::grabImage not connected!");
  }

  return result;

}  // end grabImage


// TODO: @tthomas - Implement later if needed
void PointGreyCamera::grabStereoImage(sensor_msgs::Image &image, const std::string &frame_id,
  sensor_msgs::Image &second_image, const std::string &second_frame_id)
{

}

// TODO @tthomas
// uint PointGreyCamera::getGain()
// {
//   return metadata_.embeddedGain >> 20;
// }

// uint PointGreyCamera::getShutter()
// {
//   return metadata_.embeddedShutter >> 20;
// }

// uint PointGreyCamera::getBrightness()
// {
//   return metadata_.embeddedTimeStamp >> 20;
// }

// uint PointGreyCamera::getExposure()
// {
//   return metadata_.embeddedBrightness >> 20;
// }

// uint PointGreyCamera::getWhiteBalance()
// {
//   return metadata_.embeddedExposure >> 8;
// }

// uint PointGreyCamera::getROIPosition()
// {
//   return metadata_.embeddedROIPosition >> 24;
// }

void PointGreyCamera::setDesiredCamera(const uint32_t &id)
{
  serial_ = id;
}

// TODO(tthomas)
// std::vector<uint32_t> PointGreyCamera::getAttachedCameras()
// {
//   std::vector<uint32_t> cameras;
//   unsigned int num_cameras;
//   Error error = busMgr_.GetNumOfCameras(&num_cameras);
//   PointGreyCamera::handleError("PointGreyCamera::getAttachedCameras: Could not get number of cameras", error);
//   for(unsigned int i = 0; i < num_cameras; i++)
//   {
//     unsigned int this_serial;
//     error = busMgr_.GetCameraSerialNumberFromIndex(i, &this_serial);
//     PointGreyCamera::handleError("PointGreyCamera::getAttachedCameras: Could not get get serial number from index", error);
//     cameras.push_back(this_serial);
//   }
//   return cameras;
// }



bool PointGreyCamera::setProperty(const std::string &property_name, const std::string &entry_name)
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
  Spinnaker::GenApi::CEnumerationPtr enumerationPtr = node_map_->GetNode(property_name.c_str());

  if (!Spinnaker::GenApi::IsImplemented(enumerationPtr))
  {
    ROS_ERROR_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") Enumeration name " << property_name << " not implemented.");
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

          ROS_INFO_STREAM_ONCE("[PointGreyCamera]: (" << serial_ <<  ") " << property_name << " set to " <<
            enumerationPtr->GetCurrentEntry()->GetSymbolic() << ".");

          return true;
        }
        else
        {
          ROS_WARN_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") Entry name " << entry_name << " not writable.");
        }
      }
      else
      {
        ROS_WARN_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") Entry name " << entry_name << " not available.");
      }
    }
    else
    {
      ROS_WARN_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") Enumeration " << property_name << " not writable.");
    }
  }
  else
  {
    ROS_WARN_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") Enumeration " << property_name << " not available.");
  }
  return false;
}

bool PointGreyCamera::setProperty(const std::string &property_name, const float& value)
{
  Spinnaker::GenApi::CFloatPtr floatPtr = node_map_->GetNode(property_name.c_str());

  if (!Spinnaker::GenApi::IsImplemented(floatPtr))
  {
    ROS_ERROR_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") Feature name " << property_name << " not implemented.");
    return false;
  }
  if (Spinnaker::GenApi::IsAvailable(floatPtr))
  {
    if (Spinnaker::GenApi::IsWritable(floatPtr))
    {
      floatPtr->SetValue(value);
      ROS_INFO_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") " <<  property_name << " set to " << floatPtr->GetValue() << ".");
      return true;
    } else {
      ROS_WARN_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") Feature " <<
       property_name << " not writable.");
    }
  } else {
    ROS_WARN_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") Feature " <<
      property_name << " not available.");
  }
  return false;
}


bool PointGreyCamera::setProperty(const std::string &property_name, const bool &value)
{
  Spinnaker::GenApi::CBooleanPtr boolPtr = node_map_->GetNode(property_name.c_str());
  if (!Spinnaker::GenApi::IsImplemented(boolPtr))
  {
    ROS_ERROR_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") Feature name " << property_name << " not implemented.");
    return false;
  }
  if (Spinnaker::GenApi::IsAvailable(boolPtr))
  {
    if (Spinnaker::GenApi::IsWritable(boolPtr))
    {
      boolPtr->SetValue(value);
      ROS_INFO_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") " << property_name << " set to " << boolPtr->GetValue() << ".");
      return true;
    }
    else
    {
      ROS_WARN_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") Feature " << property_name << " not writable.");
    }
  }
  else
  {
    ROS_WARN_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") Feature " <<  property_name << " not available.");
  }
  return false;
}

bool PointGreyCamera::setProperty(const std::string &property_name, const int &value)
{
  Spinnaker::GenApi::CIntegerPtr intPtr = node_map_->GetNode(property_name.c_str());
  if (!Spinnaker::GenApi::IsImplemented(intPtr))
  {
    ROS_ERROR_STREAM_ONCE("[PointGreyCamera]: (" << serial_ <<  ") Feature name " << property_name << " not implemented.");
    return false;
  }
  if (Spinnaker::GenApi::IsAvailable(intPtr))
  {
    if (Spinnaker::GenApi::IsWritable(intPtr))
    {
      intPtr->SetValue(value);
      ROS_INFO_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") " << property_name << " set to " << intPtr->GetValue() << ".");
      return true;
    }
    else
    {
      ROS_WARN_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") Feature " << property_name << " not writable.");
    }
  }
  else
  {
    ROS_WARN_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") Feature " << property_name << " not available.");
  }
  return false;
}

bool PointGreyCamera::setMaxInt(const std::string &property_name)
{
  Spinnaker::GenApi::CIntegerPtr intPtr =  node_map_->GetNode(property_name.c_str());

  if (Spinnaker::GenApi::IsAvailable(intPtr))
  {
    if (Spinnaker::GenApi::IsWritable(intPtr))
    {
      intPtr->SetValue(intPtr->GetMax());
      ROS_INFO_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") " << property_name << " set to " << intPtr->GetValue() << ".");
      return true;
    }
    else
    {
      ROS_WARN_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") Feature " << property_name << " not writable.");
    }
  }
  else
  {
    ROS_WARN_STREAM_ONCE("[PointGreyCamera]: (" << serial_ << ") Feature " << property_name << " not available.");
  }
  return false;
}


int PointGreyCamera::ConfigureChunkData(Spinnaker::GenApi::INodeMap & nodeMap)
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

    for (int i = 0; i < entries.size(); i++)
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
