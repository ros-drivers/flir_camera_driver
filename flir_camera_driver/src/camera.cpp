#include "flir_camera_driver/camera.h"

namespace flir_camera_driver
{
bool Camera::init()
{
  // Set Throughput to maximum
  //=====================================
  setMaxInt(node_map_, "DeviceLinkThroughputLimit");
  return true;
}
bool Camera::setFrameRate(const float frame_rate)
{
  // This enables the "AcquisitionFrameRateEnabled"
  //======================================

  Spinnaker::GenApi::CBooleanPtr ptrAcquisitionFrameRateEnable = node_map_->GetNode("AcquisitionFrameRateEnable");
  if (!IsAvailable(ptrAcquisitionFrameRateEnable) || !IsWritable(ptrAcquisitionFrameRateEnable))
  {
    ROS_ERROR_ONCE("Unable to enable the AcquisitionFrameRateEnable. Aborting... %u %u \n",
                   IsWritable(ptrAcquisitionFrameRateEnable), IsAvailable(ptrAcquisitionFrameRateEnable));
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
  return true;
}

bool Camera::setNewConfiguration(FlirConfig& config, const uint32_t& level)
{

  // return true if we can set values as desired.
  bool retVal = true;

  float temp_frame_rate = config.acquisition_frame_rate;
  retVal = setFrameRate(temp_frame_rate);

  // Set Trigger and Strobe Settings
  // NOTE: The trigger must be disabled (i.e. TriggerMode = "Off") in order to configure whether the source is software or hardware.
  retVal = setProperty(node_map_, "TriggerMode", std::string("Off"));
  retVal = setProperty(node_map_, "TriggerSource", config.trigger_source);
  retVal = setProperty(node_map_, "TriggerSelector", config.trigger_selector);
  retVal = setProperty(node_map_, "TriggerActivation", config.trigger_activation_mode);
  retVal = setProperty(node_map_, "TriggerMode", config.enable_trigger);

  retVal = setProperty(node_map_, "LineSelector", config.line_selector);
  retVal = setProperty(node_map_, "LineMode", config.line_mode);
  retVal = setProperty(node_map_, "LineSource", config.line_source);

  // Set auto exposure
  retVal = setProperty(node_map_, "ExposureMode", config.exposure_mode);
  retVal = setProperty(node_map_, "ExposureAuto", config.exposure_auto);


  // Set Video Mode, Image and Pixel formats
  // retVal = FlirCamera::setVideoMode(config.video_mode);
  // retVal = FlirCamera::setImageControlFormats(config);

  /*
  TODO @tthomas: Revisit/Debug setProperty method for setting frame rate and other properties
  retVal = setProperty(node_map_, "AcquisitionFrameRateAuto", "Off");
  retVal = setProperty(node_map_, "AcquisitionFrameRateEnabled", true);


  // retVal = setProperty(node_map_, "AcquisitionFrameRate", config.acquisition_frame_rate);
  // TODO @tthomas: streamline double& to float& conversions
  float temp_frame_rate = config.acquisition_frame_rate;
  retVal = setProperty(node_map_, "AcquisitionFrameRate", temp_frame_rate);  // Feature AcquisitionFrameRate not writable.
  */


  // Set sharpness
  if (config.sharpening_enable)
  {
    retVal = setProperty(node_map_, "SharpeningAuto", config.auto_sharpness);
    // retVal = setProperty(node_map_, "Sharpening", config.sharpness);
    float temp_sharpness = config.sharpness;
    float temp_sharpening_threshold = config.sharpening_threshold;
    retVal = setProperty(node_map_, "Sharpening", temp_sharpness);
    retVal = setProperty(node_map_, "SharpeningThreshold", temp_sharpening_threshold);
  }

  // Set saturation
  if (config.saturation_enable)
  {
    retVal = setProperty(node_map_, "SaturationEnable", config.saturation_enable);
    float temp_saturation = config.saturation;
    retVal = setProperty(node_map_, "Saturation", temp_saturation);
  }


  // Set shutter time/speed
  if (config.exposure_auto.compare(std::string("Off")) == 0)
  {
    float temp_exposure_time = config.exposure_time;
    retVal = setProperty(node_map_, "ExposureTime", temp_exposure_time);
  }

  float temp_auto_exposure_exposure_time_upper_limit= config.auto_exposure_time_upper_limit;
  retVal = setProperty(node_map_, "AutoExposureTimeUpperLimit", temp_auto_exposure_exposure_time_upper_limit);


  // Set gain
  retVal = setProperty(node_map_, "GainSelector", config.gain_selector);
  retVal = setProperty(node_map_, "GainAuto", config.auto_gain);

  float temp_gain = config.gain;
  retVal = setProperty(node_map_, "Gain", temp_gain);

  // Set brightness
  float temp_brightness = config.brightness;
  retVal = setProperty(node_map_, "BlackLevel", temp_brightness);

  // Set gamma
  if (config.gamma_enable)
  {
    retVal = setProperty(node_map_, "GammaEnable", config.gamma_enable);

    float temp_gamma = config.gamma;
    retVal = setProperty(node_map_, "Gamma", temp_gamma);
  }

  // Set white balance
  retVal = setProperty(node_map_, "BalanceWhiteAuto", config.auto_white_balance);
  retVal = setProperty(node_map_, "BalanceRatioSelector", "Blue");

  float temp_white_balance_blue_ratio = config.white_balance_blue_ratio;
  retVal = setProperty(node_map_, "BalanceRatio", temp_white_balance_blue_ratio);

  retVal = setProperty(node_map_, "BalanceRatioSelector", "Red");
  float temp_white_balance_red_ratio = config.white_balance_red_ratio;
  retVal = setProperty(node_map_, "BalanceRatio", temp_white_balance_red_ratio);



  return retVal;
}

bool Camera::setVideoMode(const std::string& videoMode)
{

  ROS_INFO_STREAM_ONCE("\n\n videoMode: " << videoMode << "\n\n");

  // return true if we can set the video mode as desired.
  bool retVal = true;

  if (videoMode.compare("1280x960") == 0)
  {
    retVal = setProperty(node_map_, "VideoMode", "Mode0");
  }
  else if (videoMode.compare("640x480_pixel_aggregation") == 0)
  {
    retVal = setProperty(node_map_, "VideoMode", "Mode1");
  }
  else if (videoMode.compare("640x480_pixel_decimation") == 0)
  {
    retVal = setProperty(node_map_, "VideoMode", "Mode4");
  }
  else if (videoMode.compare("320x240") == 0)
  {
    retVal = setProperty(node_map_, "VideoMode", "Mode5");
  }
  else
  {
    ROS_ERROR_ONCE("Video Mode Unknown!");
    retVal = false;
  }

  return retVal;
}


// Image Size and Pixel Format
bool Camera::setImageControlFormats(flir_camera_driver::FlirConfig &config)
{

  // return true if we can set values as desired.
  bool retVal = true;

  // Apply minimum to offset X
  retVal = setProperty(node_map_, "OffsetX", config.image_format_x_offset);
  // Apply minimum to offset Y
  retVal = setProperty(node_map_, "OffsetY", config.image_format_y_offset);

  // Set maximum width
  retVal = setProperty(node_map_, "Width", config.image_format_roi_width);
  retVal = setProperty(node_map_, "Height", config.image_format_roi_height);

  // Set Pixel Format
  retVal = setProperty(node_map_, "PixelFormat", config.image_format_color_coding);

  return retVal;
}

/*
void Camera::setGigEParameters(bool auto_packet_size, unsigned int packet_size, unsigned int packet_delay)
{
}

void Camera::setupGigEPacketSize(PGRGuid & guid)
{
}

void Camera::setupGigEPacketSize(PGRGuid & guid, unsigned int packet_size)
{

}

void Camera::setupGigEPacketDelay(PGRGuid & guid, unsigned int packet_delay)
{
}

*/

// uint FlirCamera::getGain()
// {
//   return metadata_.embeddedGain >> 20;
// }

// uint Camera::getShutter()
// {
//   return metadata_.embeddedShutter >> 20;
// }

// uint Camera::getBrightness()
// {
//   return metadata_.embeddedTimeStamp >> 20;
// }

// uint Camera::getExposure()
// {
//   return metadata_.embeddedBrightness >> 20;
// }

// uint Camera::getWhiteBalance()
// {
//   return metadata_.embeddedExposure >> 8;
// }

// uint Camera::getROIPosition()
// {
//   return metadata_.embeddedROIPosition >> 24;
// }

//float Camera::getCameraTemperature()
//{
//}

//float Camera::getCameraFrameRate()
//{
//}

Camera::Camera(Spinnaker::GenApi::INodeMap* node_map)
{
  node_map_ = node_map;
  init();
}
}
