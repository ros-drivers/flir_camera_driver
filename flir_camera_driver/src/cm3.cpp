#include "flir_camera_driver/cm3.h"

namespace flir_camera_driver
{
bool Cm3::setFrameRate(const float frame_rate)
{
  bool retVal = true;
  // This enables the "AcquisitionFrameRateEnabled"
  //======================================
  retVal &= setProperty(node_map_, "AcquisitionFrameRateEnabled", true);  // different from Bfly S

  // This sets the "AcquisitionFrameRateAuto" to "Off"
  //======================================
  retVal &= setProperty(node_map_, "AcquisitionFrameRateAuto", "Off");  // different from Bfly S

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

bool Cm3::setNewConfiguration(FlirConfig& config, const uint32_t& level)
{
  // return true if we can set values as desired.
  bool retVal = true;

  try
  {
    if (level >= LEVEL_RECONFIGURE_STOP)
      retVal &= setImageControlFormats(config);

    retVal &= setFrameRate(static_cast<float>(config.acquisition_frame_rate));
    retVal &= setProperty(node_map_, "AcquisitionFrameRateEnabled",
                          config.acquisition_frame_rate_enable);  // Set enable after frame rate encase its false

    // Set Trigger and Strobe Settings
    // NOTE: The trigger must be disabled (i.e. TriggerMode = "Off") in order to configure whether the source is
    // software or hardware.
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

    // Set Video Mode, Image and Pixel formats
    // retVal = FlirCamera::setImageControlFormats(config);

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
      retVal &= setProperty(node_map_, "AutoExposureTimeUpperLimit",
                            static_cast<float>(config.auto_exposure_time_upper_limit));  // Different than BFly S
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
      if (config.auto_white_balance.compare(std::string("Off")) == 0)
      {
        retVal &= setProperty(node_map_, "BalanceRatioSelector", "Blue");
        retVal &= setProperty(node_map_, "BalanceRatio", static_cast<float>(config.white_balance_blue_ratio));
        retVal &= setProperty(node_map_, "BalanceRatioSelector", "Red");
        retVal &= setProperty(node_map_, "BalanceRatio", static_cast<float>(config.white_balance_red_ratio));
      }
    }
  }
  catch (Spinnaker::Exception& e)
  {
    ROS_ERROR("Spinnaker Exception: %s", e.what());
    return false;
  }
  return retVal;
}
}
