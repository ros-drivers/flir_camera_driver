#include "flir_camera_driver/camera.h"

namespace flir_camera_driver
{
bool Camera::init()
{
  Spinnaker::GenApi::CIntegerPtr height_max_ptr = node_map_->GetNode("HeightMax");
  if (!IsAvailable(height_max_ptr) || !IsReadable(height_max_ptr))
  {
    ROS_ERROR_ONCE("Unable to read HeightMax");
    return false;
  }
  height_max_ = height_max_ptr->GetValue();
  Spinnaker::GenApi::CIntegerPtr width_max_ptr = node_map_->GetNode("WidthMax");
  if (!IsAvailable(width_max_ptr) || !IsReadable(width_max_ptr))
  {
    ROS_ERROR_ONCE("Unable to read WidthMax");
    return false;
  }
  width_max_ = width_max_ptr->GetValue();
  // Set Throughput to maximum
  //=====================================
  setMaxInt(node_map_, "DeviceLinkThroughputLimit");
  return true;
}
bool Camera::setFrameRate(const float frame_rate)
{
  bool retVal = true;
  // This enables the "AcquisitionFrameRateEnabled"
  //======================================
  retVal &= setProperty(node_map_, "AcquisitionFrameRateEnable", true);

  // This sets the "AcquisitionFrameRate" to X FPS
  // ========================================

  Spinnaker::GenApi::CFloatPtr ptrAcquisitionFrameRate = node_map_->GetNode("AcquisitionFrameRate");
  ROS_DEBUG_STREAM("Minimum Frame Rate: \t " << ptrAcquisitionFrameRate->GetMin());
  ROS_DEBUG_STREAM("Maximum Frame rate: \t " << ptrAcquisitionFrameRate->GetMax());

  // Finally Set the Frame Rate
  retVal &= setProperty(node_map_, "AcquisitionFrameRate", frame_rate);

  ROS_DEBUG_STREAM("Current Frame rate: \t " << ptrAcquisitionFrameRate->GetValue());
  return retVal;
}

bool Camera::setNewConfiguration(FlirConfig& config, const uint32_t& level)
{
  // return true if we can set values as desired.
  bool retVal = true;
try
  {
    if(level >= LEVEL_RECONFIGURE_STOP)
      retVal &= setImageControlFormats(config);

    retVal &= setFrameRate(static_cast<float>(config.acquisition_frame_rate));
    // Set enable after frame rate encase its false
    retVal &= setProperty(node_map_, "AcquisitionFrameRateEnable", config.acquisition_frame_rate_enable);

    // Set Trigger and Strobe Settings
    // NOTE: The trigger must be disabled (i.e. TriggerMode = "Off") in order to configure whether the source is software or hardware.
    retVal &= setProperty(node_map_, "TriggerMode", std::string("Off"));
    retVal &= setProperty(node_map_, "TriggerSource", config.trigger_source);
    retVal &= setProperty(node_map_, "TriggerSelector", config.trigger_selector);
    retVal &= setProperty(node_map_, "TriggerActivation", config.trigger_activation_mode);
    retVal &= setProperty(node_map_, "TriggerMode", config.enable_trigger);

    retVal &= setProperty(node_map_, "LineSelector", config.line_selector);
    retVal &= setProperty(node_map_, "LineMode", config.line_mode);
    retVal &= setProperty(node_map_, "LineSource", config.line_source);

    // Set auto exposure
    retVal &= setProperty(node_map_, "ExposureMode", config.exposure_mode);
    retVal &= setProperty(node_map_, "ExposureAuto", config.exposure_auto);

    // Set sharpness
    if (IsAvailable(node_map_->GetNode("SharpeningEnable")))
    {
      retVal &= setProperty(node_map_, "SharpeningEnable", config.sharpening_enable);
      if (config.sharpening_enable)
      {
        retVal &= setProperty(node_map_, "SharpeningAuto", config.auto_sharpness);
        retVal &= setProperty(node_map_, "Sharpening", static_cast<float>(config.sharpness));
        retVal &= setProperty(node_map_, "SharpeningThreshold", static_cast<float>(config.sharpening_threshold));
      }
    }

    // Set saturation
    if (IsAvailable(node_map_->GetNode("SaturationEnable")))
    {
    retVal &= setProperty(node_map_, "SaturationEnable", config.saturation_enable);
      if (config.saturation_enable)
      {
        retVal &= setProperty(node_map_, "Saturation", static_cast<float>(config.saturation));
      }
    }

    // Set shutter time/speed
    if (config.exposure_auto.compare(std::string("Off")) == 0)
    {
      retVal &= setProperty(node_map_, "ExposureTime", static_cast<float>(config.exposure_time));
    }
    else
    {
      retVal &= setProperty(node_map_, "AutoExposureExposureTimeUpperLimit", static_cast<float>(config.auto_exposure_time_upper_limit));
    }

    // Set gain
    retVal &= setProperty(node_map_, "GainSelector", config.gain_selector);
    retVal &= setProperty(node_map_, "GainAuto", config.auto_gain);
    if (config.auto_gain.compare(std::string("Off")) == 0)
    {
      retVal &= setProperty(node_map_, "Gain", static_cast<float>(config.gain));
    }

    // Set brightness
    retVal &= setProperty(node_map_, "BlackLevel", static_cast<float>(config.brightness));

    // Set gamma
    if (config.gamma_enable)
    {
      retVal &= setProperty(node_map_, "GammaEnable", config.gamma_enable);
      retVal &= setProperty(node_map_, "Gamma", static_cast<float>(config.gamma));
    }

    // Set white balance
    if (IsAvailable(node_map_->GetNode("BalanceWhiteAuto")))
    {
      retVal &= setProperty(node_map_, "BalanceWhiteAuto", config.auto_white_balance);
      if(config.auto_white_balance.compare(std::string("Off")) == 0)
      {
        retVal &= setProperty(node_map_, "BalanceRatioSelector", "Blue");
        retVal &= setProperty(node_map_, "BalanceRatio", static_cast<float>(config.white_balance_blue_ratio));
        retVal &= setProperty(node_map_, "BalanceRatioSelector", "Red");
        retVal &= setProperty(node_map_, "BalanceRatio", static_cast<float>(config.white_balance_red_ratio));
      }
    }
  }
  catch(Spinnaker::Exception & e)
  {
    ROS_ERROR("Spinnaker Exception: %s", e.what());
    return false;
  }
  return retVal;
}


// Image Size and Pixel Format
bool Camera::setImageControlFormats(flir_camera_driver::FlirConfig &config)
{
  // return true if we can set values as desired.
  bool retVal = true;

  // Set Binning and Decimation
  retVal &= setProperty(node_map_, "BinningHorizontal", config.image_format_x_binning);
  retVal &= setProperty(node_map_, "BinningVertical", config.image_format_y_binning);
  retVal &= setProperty(node_map_, "DecimationHorizontal", config.image_format_x_decimation);
  retVal &= setProperty(node_map_, "DecimationVertical", config.image_format_y_decimation);

  // Grab the Max values after decimation
  Spinnaker::GenApi::CIntegerPtr height_max_ptr = node_map_->GetNode("HeightMax");
  if (!IsAvailable(height_max_ptr) || !IsReadable(height_max_ptr))
  {
    ROS_ERROR_ONCE("Unable to read HeightMax");
    return false;
  }
  height_max_ = height_max_ptr->GetValue();
  Spinnaker::GenApi::CIntegerPtr width_max_ptr = node_map_->GetNode("WidthMax");
  if (!IsAvailable(width_max_ptr) || !IsReadable(width_max_ptr))
  {
    ROS_ERROR_ONCE("Unable to read WidthMax");
    return false;
  }
  width_max_ = width_max_ptr->GetValue();

  // Offset first encase expanding ROI
  // Apply offset X
  retVal &= setProperty(node_map_, "OffsetX", 0);
  // Apply offset Y
  retVal &= setProperty(node_map_, "OffsetY", 0);

  // Set Width/Height
  if(config.image_format_roi_width <= 0 || config.image_format_roi_width > width_max_)
    retVal &= setProperty(node_map_, "Width", width_max_);
  else
    retVal &= setProperty(node_map_, "Width", config.image_format_roi_width);
  if(config.image_format_roi_height <= 0 || config.image_format_roi_height > height_max_)
    retVal &= setProperty(node_map_, "Height", height_max_);
  else
    retVal &= setProperty(node_map_, "Height", config.image_format_roi_height);

  // Apply offset X
  retVal &= setProperty(node_map_, "OffsetX", config.image_format_x_offset);
  // Apply offset Y
  retVal &= setProperty(node_map_, "OffsetY", config.image_format_y_offset);

  // Set Pixel Format
  retVal &= setProperty(node_map_, "PixelFormat", config.image_format_color_coding);

  return retVal;
}

void Camera::setGain(const float& gain)
{
  setProperty(node_map_, "GainAuto", "Off");
  setProperty(node_map_, "Gain", static_cast<float>(gain));
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


 uint Camera::getHeightMax()
 {
   return height_max_;
 }

 uint Camera::getWidthMax()
 {
   return width_max_;
 }

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
