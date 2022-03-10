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

/**
   @file nodelet.cpp
   @author Chad Rockey
   @date July 13, 2011
   @brief ROS nodelet for the Point Grey Chameleon Camera

   @attention Copyright (C) 2011
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

/**
   @file nodelet.cpp
   @author Teyvonia Thomas
   @date August 28, 2017
   @brief ROS nodelet for the Point Grey Chameleon Camera - Updated to use Spinnaker driver insteady of Flycapture
*/

// ROS and associated nodelet interface and PLUGINLIB declaration header
#include "ros/ros.h"
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "spinnaker_camera_driver/SpinnakerCamera.h"  // The actual standalone library for the Spinnakers
#include "spinnaker_camera_driver/diagnostics.h"

#include <image_transport/image_transport.h>          // ROS library that allows sending compressed images
#include <camera_info_manager/camera_info_manager.h>  // ROS library that publishes CameraInfo topics
#include <sensor_msgs/CameraInfo.h>                   // ROS message header for CameraInfo

#include <wfov_camera_msgs/WFOVImage.h>
#include <image_exposure_msgs/ExposureSequence.h>  // Message type for configuring gain and white balance.

#include <diagnostic_updater/diagnostic_updater.h>  // Headers for publishing diagnostic messages.
#include <diagnostic_updater/publisher.h>

#include <boost/thread.hpp>  // Needed for the nodelet to launch the reading thread.

#include <dynamic_reconfigure/server.h>  // Needed for the dynamic_reconfigure gui service to run

#include <fstream>
#include <memory>
#include <string>
#include <utility>

namespace spinnaker_camera_driver
{
class SpinnakerCameraNodelet : public nodelet::Nodelet
{
public:
  SpinnakerCameraNodelet()
  {
  }

  ~SpinnakerCameraNodelet()
  {
    std::lock_guard<std::mutex> scopedLock(connect_mutex_);

    if (diagThread_)
    {
      diagThread_->interrupt();
      diagThread_->join();
    }

    if (pubThread_)
    {
      pubThread_->interrupt();
      pubThread_->join();

      try
      {
        NODELET_DEBUG_ONCE("Stopping camera capture.");
        spinnaker_.stop();
        NODELET_DEBUG_ONCE("Disconnecting from camera.");
        spinnaker_.disconnect();
      }
      catch (const std::runtime_error& e)
      {
        NODELET_ERROR("%s", e.what());
      }
    }
  }

private:
  /*!
  * \brief Function that allows reconfiguration of the camera.
  *
  * This function serves as a callback for the dynamic reconfigure service.  It simply passes the configuration object
  * to the driver to allow the camera to reconfigure.
  * \param config  camera_library::CameraConfig object passed by reference.  Values will be changed to those the driver
  * is currently using.
  * \param level driver_base reconfiguration level.  See driver_base/SensorLevels.h for more information.
  */

  void paramCallback(const spinnaker_camera_driver::SpinnakerConfig& config, uint32_t level)
  {
    config_ = config;

    try
    {
      NODELET_DEBUG_ONCE("Dynamic reconfigure callback with level: %u", level);
      spinnaker_.setNewConfiguration(config, level);

      // Store needed parameters for the metadata message
      gain_ = config.gain;
      wb_blue_ = config.white_balance_blue_ratio;
      wb_red_ = config.white_balance_red_ratio;

      // No separate param in CameraInfo for binning/decimation
      binning_x_ = config.image_format_x_binning * config.image_format_x_decimation;
      binning_y_ = config.image_format_y_binning * config.image_format_y_decimation;

      // Store CameraInfo RegionOfInterest information
      // TODO(mhosmar): Not compliant with CameraInfo message: "A particular ROI always denotes the
      //                same window of pixels on the camera sensor, regardless of binning settings."
      //                These values are in the post binned frame.
      if ((config.image_format_roi_width + config.image_format_roi_height) > 0 &&
          (config.image_format_roi_width < spinnaker_.getWidthMax() ||
           config.image_format_roi_height < spinnaker_.getHeightMax()))
      {
        roi_x_offset_ = config.image_format_x_offset;
        roi_y_offset_ = config.image_format_y_offset;
        roi_width_ = config.image_format_roi_width;
        roi_height_ = config.image_format_roi_height;
        do_rectify_ = true;  // Set to true if an ROI is used.
      }
      else
      {
        // Zeros mean the full resolution was captured.
        roi_x_offset_ = 0;
        roi_y_offset_ = 0;
        roi_height_ = 0;
        roi_width_ = 0;
        do_rectify_ = false;  // Set to false if the whole image is captured.
      }
    }
    catch (std::runtime_error& e)
    {
      NODELET_ERROR("Reconfigure Callback failed with error: %s", e.what());
    }
  }

  void diagCb()
  {
    if (!diagThread_)  // We need to connect
    {
      // Start the thread to loop through and publish messages
      diagThread_.reset(
          new boost::thread(boost::bind(&spinnaker_camera_driver::SpinnakerCameraNodelet::diagPoll, this)));
    }
  }

  /*!
  * \brief Connection callback to only do work when someone is listening.
  *
  * This function will connect/disconnect from the camera depending on who is using the output.
  */
  void connectCb()
  {
    if (!pubThread_)  // We need to connect
    {
      // Start the thread to loop through and publish messages
      pubThread_.reset(
          new boost::thread(boost::bind(&spinnaker_camera_driver::SpinnakerCameraNodelet::devicePoll, this)));
    }

    // @tthomas - removing subscriber check and logic below as it's leading to mutex locks and crashes currently
    /*
    NODELET_DEBUG_ONCE("Connect callback!");
    std::lock_guard<std::mutex> scopedLock(connect_mutex_); // Grab the mutex.  Wait until we're done initializing
    before letting this function through.
    // Check if we should disconnect (there are 0 subscribers to our data)
    if(it_pub_.getNumSubscribers() == 0 && pub_->getPublisher().getNumSubscribers() == 0)
    {
      if (pubThread_)
      {
        NODELET_DEBUG_ONCE("Disconnecting.");
        pubThread_->interrupt();
        scopedLock.unlock();
        pubThread_->join();
        scopedLock.lock();
        pubThread_.reset();
        sub_.shutdown();

        try
        {
          NODELET_DEBUG_ONCE("Stopping camera capture.");
          spinnaker_.stop();
        }
        catch(std::runtime_error& e)
        {
          NODELET_ERROR("%s", e.what());
        }

        try
        {
          NODELET_DEBUG_ONCE("Disconnecting from camera.");
          spinnaker_.disconnect();
        }
        catch(std::runtime_error& e)
        {
          NODELET_ERROR("%s", e.what());
        }
      }
    }
    else if(!pubThread_)     // We need to connect
    {
      // Start the thread to loop through and publish messages
      pubThread_.reset(new boost::thread(boost::bind(&spinnaker_camera_driver::SpinnakerCameraNodelet::devicePoll,
    this)));
    }
    else
    {
      NODELET_DEBUG_ONCE("Do nothing in callback.");
    }
    */
  }

  /*!
  * \brief Serves as a psuedo constructor for nodelets.
  *
  * This function needs to do the MINIMUM amount of work to get the nodelet running.  Nodelets should not call blocking
  * functions here.
  */
  void onInit()
  {
    // Get nodeHandles
    ros::NodeHandle& nh = getMTNodeHandle();
    ros::NodeHandle& pnh = getMTPrivateNodeHandle();

    // Get a serial number through ros
    int serial = 0;

    XmlRpc::XmlRpcValue serial_xmlrpc;
    pnh.getParam("serial", serial_xmlrpc);
    if (serial_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      pnh.param<int>("serial", serial, 0);
    }
    else if (serial_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      std::string serial_str;
      pnh.param<std::string>("serial", serial_str, "0");
      std::istringstream(serial_str) >> serial;
    }
    else
    {
      NODELET_DEBUG_ONCE("Serial XMLRPC type.");
      serial = 0;
    }

    std::string camera_serial_path;
    pnh.param<std::string>("camera_serial_path", camera_serial_path, "");
    NODELET_DEBUG_ONCE("Camera serial path %s", camera_serial_path.c_str());
    // If serial has been provided directly as a param, ignore the path
    // to read in the serial from.
    while (serial == 0 && !camera_serial_path.empty())
    {
      serial = readSerialAsHexFromFile(camera_serial_path);
      if (serial == 0)
      {
        NODELET_WARN_ONCE("Waiting for camera serial path to become available");
        ros::Duration(1.0).sleep();  // Sleep for 1 second, wait for serial device path to become available
      }
    }

    NODELET_DEBUG_ONCE("Using camera serial %d", serial);

    spinnaker_.setDesiredCamera((uint32_t)serial);

    // Get GigE camera parameters:
    pnh.param<int>("packet_size", packet_size_, 1400);
    pnh.param<bool>("auto_packet_size", auto_packet_size_, true);
    pnh.param<int>("packet_delay", packet_delay_, 4000);

    // TODO(mhosmar):  Set GigE parameters:
    // spinnaker_.setGigEParameters(auto_packet_size_, packet_size_, packet_delay_);

    // Get the location of our camera config yaml
    std::string camera_info_url;
    pnh.param<std::string>("camera_info_url", camera_info_url, "");
    // Get the desired frame_id, set to 'camera' if not found
    pnh.param<std::string>("frame_id", frame_id_, "camera");
    // Do not call the connectCb function until after we are done initializing.
    std::lock_guard<std::mutex> scopedLock(connect_mutex_);

    // Start up the dynamic_reconfigure service, note that this needs to stick around after this function ends
    srv_ = std::make_shared<dynamic_reconfigure::Server<spinnaker_camera_driver::SpinnakerConfig> >(pnh);
    dynamic_reconfigure::Server<spinnaker_camera_driver::SpinnakerConfig>::CallbackType f =
        boost::bind(&spinnaker_camera_driver::SpinnakerCameraNodelet::paramCallback, this, _1, _2);

    srv_->setCallback(f);

    // queue size of ros publisher
    int queue_size;
    pnh.param<int>("queue_size", queue_size, 5);

    // Start the camera info manager and attempt to load any configurations
    std::stringstream cinfo_name;
    cinfo_name << serial;
    cinfo_.reset(new camera_info_manager::CameraInfoManager(nh, cinfo_name.str(), camera_info_url));

    // Publish topics using ImageTransport through camera_info_manager (gives cool things like compression)
    it_.reset(new image_transport::ImageTransport(nh));
    image_transport::SubscriberStatusCallback cb = boost::bind(&SpinnakerCameraNodelet::connectCb, this);
    it_pub_ = it_->advertiseCamera("image_raw", queue_size, cb, cb);

    // Set up diagnostics
    updater_.setHardwareID("spinnaker_camera " + cinfo_name.str());

    // Set up a diagnosed publisher
    double desired_freq;
    std::string device_type;
    pnh.param<std::string>("device_type", device_type, "USB3");
    pnh.param<double>("desired_freq", desired_freq, 30.0);
    pnh.param<double>("min_freq", min_freq_, desired_freq);
    pnh.param<double>("max_freq", max_freq_, desired_freq);
    double freq_tolerance;  // Tolerance before stating error on publish frequency, fractional percent of desired
                            // frequencies.
    pnh.param<double>("freq_tolerance", freq_tolerance, 0.1);
    int window_size;  // Number of samples to consider in frequency
    pnh.param<int>("window_size", window_size, 30);
    double min_acceptable;  // The minimum publishing delay (in seconds) before warning.  Negative values mean future
                            // dated messages.
    pnh.param<double>("min_acceptable_delay", min_acceptable, 0.0);
    double max_acceptable;  // The maximum publishing delay (in seconds) before warning.
    pnh.param<double>("max_acceptable_delay", max_acceptable, 0.2);
    ros::SubscriberStatusCallback cb2 = boost::bind(&SpinnakerCameraNodelet::connectCb, this);
    pub_.reset(new diagnostic_updater::DiagnosedPublisher<wfov_camera_msgs::WFOVImage>(
        nh.advertise<wfov_camera_msgs::WFOVImage>("image", queue_size, cb2, cb2),
        updater_,
        diagnostic_updater::FrequencyStatusParam(&min_freq_,
                                                 &max_freq_,
                                                 freq_tolerance,
                                                 window_size),
        diagnostic_updater::TimeStampStatusParam(min_acceptable,
                                                 max_acceptable)));

    // Set up diagnostics aggregator publisher and diagnostics manager
    ros::SubscriberStatusCallback diag_cb = boost::bind(&SpinnakerCameraNodelet::diagCb, this);
    diagnostics_pub_.reset(new ros::Publisher(
        nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1, diag_cb, diag_cb)));

    diag_man = std::unique_ptr<DiagnosticsManager>(new DiagnosticsManager(
        frame_id_, std::to_string(spinnaker_.getSerial()), diagnostics_pub_, nh));
    diag_man->addDiagnostic("DeviceTemperature", true, std::make_pair(0.0f, 90.0f), -10.0f, 95.0f);
    diag_man->addDiagnostic("AcquisitionResultingFrameRate", true, std::make_pair(10.0f, 60.0f), 5.0f, 90.0f);
    diag_man->addDiagnostic("PowerSupplyVoltage", true, std::make_pair(4.5f, 5.2f), 4.4f, 5.3f);
    diag_man->addDiagnostic("PowerSupplyCurrent", true, std::make_pair(0.4f, 0.6f), 0.3f, 1.0f);
    diag_man->addDiagnostic<int>("DeviceUptime");
    if (device_type.compare("USB3") == 0 )
    {
      diag_man->addDiagnostic<int>("U3VMessageChannelID");
    }
  }

  /**
   * @brief Reads in the camera serial from a specified file path.
   * The format of the serial is expected to be base 16.
   * @param camera_serial_path The path of where to read in the serial from. Generally this
   * is a USB device path to the serial file.
   * @return int The serial number for the given path, 0 if failure.
   */
  int readSerialAsHexFromFile(std::string camera_serial_path)
  {
    NODELET_DEBUG_ONCE("Reading camera serial file from: %s", camera_serial_path.c_str());

    std::ifstream serial_file(camera_serial_path.c_str());
    std::stringstream buffer;
    int serial = 0;

    if (serial_file.is_open())
    {
      std::string serial_str((std::istreambuf_iterator<char>(serial_file)), std::istreambuf_iterator<char>());
      NODELET_DEBUG_ONCE("Serial file contents: %s", serial_str.c_str());
      buffer << std::hex << serial_str;
      buffer >> serial;
      NODELET_DEBUG_ONCE("Serial discovered %d", serial);

      return serial;
    }

    NODELET_WARN_ONCE("Unable to open serial path: %s", camera_serial_path.c_str());
    return 0;
  }

  void diagPoll()
  {
    diag_man->addAnalyzers();
    while (!boost::this_thread::interruption_requested())  // Block until we need
                                                           // to stop this
                                                           // thread.
    {
      diag_man->processDiagnostics(&spinnaker_);
    }
  }

  /*!
  * \brief Function for the boost::thread to grabImages and publish them.
  *
  * This function continues until the thread is interupted.  Responsible for getting sensor_msgs::Image and publishing
  * them.
  */
  void devicePoll()
  {
    ROS_INFO_ONCE("devicePoll");

    enum State
    {
      NONE,
      ERROR,
      STOPPED,
      DISCONNECTED,
      CONNECTED,
      STARTED
    };

    State state = DISCONNECTED;
    State previous_state = NONE;

    while (!boost::this_thread::interruption_requested())  // Block until we need to stop this thread.
    {
      bool state_changed = state != previous_state;

      previous_state = state;

      switch (state)
      {
        case ERROR:
// Generally there's no need to stop before disconnecting after an
// error. Indeed, stop will usually fail.
#if STOP_ON_ERROR
          // Try stopping the camera
          {
            std::lock_guard<std::mutex> scopedLock(connect_mutex_);
            sub_.shutdown();
          }

          try
          {
            NODELET_DEBUG_ONCE("Stopping camera.");
            spinnaker_.stop();
            NODELET_DEBUG_ONCE("Stopped camera.");

            state = STOPPED;
          }
          catch (std::runtime_error& e)
          {
            if (state_changed)
            {
              NODELET_ERROR("Failed to stop with error: %s", e.what());
              ros::Duration(1.0).sleep();  // sleep for one second each time
            }
            state = ERROR;
          }

          break;
#endif
        case STOPPED:
          // Try disconnecting from the camera
          try
          {
            NODELET_DEBUG("Disconnecting from camera.");
            spinnaker_.disconnect();
            NODELET_DEBUG("Disconnected from camera.");

            state = DISCONNECTED;
          }
          catch (std::runtime_error& e)
          {
            if (state_changed)
            {
              NODELET_ERROR("Failed to disconnect with error: %s", e.what());
              ros::Duration(1.0).sleep();  // sleep for one second each time
            }
            state = ERROR;
          }

          break;
        case DISCONNECTED:
          // Try connecting to the camera
          try
          {
            NODELET_DEBUG("Connecting to camera.");

            spinnaker_.connect();

            NODELET_DEBUG("Connected to camera.");

            // Set last configuration, forcing the reconfigure level to stop
            spinnaker_.setNewConfiguration(config_, SpinnakerCamera::LEVEL_RECONFIGURE_STOP);

            // Set the timeout for grabbing images.
            try
            {
              double timeout;
              getMTPrivateNodeHandle().param("timeout", timeout, 1.0);

              NODELET_DEBUG_ONCE("Setting timeout to: %f.", timeout);
              spinnaker_.setTimeout(timeout);
            }
            catch (const std::runtime_error& e)
            {
              NODELET_ERROR("%s", e.what());
            }

            // Subscribe to gain and white balance changes
            {
              std::lock_guard<std::mutex> scopedLock(connect_mutex_);
              sub_ =
                  getMTNodeHandle().subscribe("image_exposure_sequence", 10,
                                              &spinnaker_camera_driver::SpinnakerCameraNodelet::gainWBCallback, this);
            }
            state = CONNECTED;
          }
          catch (const std::runtime_error& e)
          {
            if (state_changed)
            {
              NODELET_ERROR("Failed to connect with error: %s", e.what());
              ros::Duration(1.0).sleep();  // sleep for one second each time
            }
            state = ERROR;
          }

          break;
        case CONNECTED:
          // Try starting the camera
          try
          {
            NODELET_DEBUG("Starting camera.");
            spinnaker_.start();
            NODELET_DEBUG("Started camera.");
            NODELET_DEBUG("Attention: if nothing subscribes to the camera topic, the camera_info is not published "
                          "on the correspondent topic.");
            state = STARTED;
          }
          catch (std::runtime_error& e)
          {
            if (state_changed)
            {
              NODELET_ERROR("Failed to start with error: %s", e.what());
              ros::Duration(1.0).sleep();  // sleep for one second each time
            }
            state = ERROR;
          }

          break;
        case STARTED:
          try
          {
            wfov_camera_msgs::WFOVImagePtr wfov_image(new wfov_camera_msgs::WFOVImage);
            // Get the image from the camera library
            NODELET_DEBUG_ONCE("Starting a new grab from camera with serial {%d}.", spinnaker_.getSerial());
            spinnaker_.grabImage(&wfov_image->image, frame_id_);

            // Set other values
            wfov_image->header.frame_id = frame_id_;

            wfov_image->gain = gain_;
            wfov_image->white_balance_blue = wb_blue_;
            wfov_image->white_balance_red = wb_red_;

            // wfov_image->temperature = spinnaker_.getCameraTemperature();

            ros::Time time = ros::Time::now() + ros::Duration(config_.time_offset);
            wfov_image->header.stamp = time;
            wfov_image->image.header.stamp = time;

            // Set the CameraInfo message
            ci_.reset(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
            ci_->header.stamp = wfov_image->image.header.stamp;
            ci_->header.frame_id = wfov_image->header.frame_id;
            // The height, width, distortion model, and parameters are all filled in by camera info manager.
            ci_->binning_x = binning_x_;
            ci_->binning_y = binning_y_;
            ci_->roi.x_offset = roi_x_offset_;
            ci_->roi.y_offset = roi_y_offset_;
            ci_->roi.height = roi_height_;
            ci_->roi.width = roi_width_;
            ci_->roi.do_rectify = do_rectify_;

            wfov_image->info = *ci_;

            // Publish the full message
            pub_->publish(wfov_image);

            // Publish the message using standard image transport
            if (it_pub_.getNumSubscribers() > 0)
            {
              sensor_msgs::ImagePtr image(new sensor_msgs::Image(wfov_image->image));
              it_pub_.publish(image, ci_);
            }
          }
          catch (CameraTimeoutException& e)
          {
            NODELET_WARN("%s", e.what());
          }

          catch (std::runtime_error& e)
          {
            NODELET_ERROR("%s", e.what());
            state = ERROR;
          }

          break;
        default:
          NODELET_ERROR("Unknown camera state %d!", state);
      }

      // Update diagnostics
      updater_.update();
    }
    NODELET_DEBUG_ONCE("Leaving thread.");
  }

  void gainWBCallback(const image_exposure_msgs::ExposureSequence& msg)
  {
    try
    {
      NODELET_DEBUG_ONCE("Gain callback:  Setting gain to %f and white balances to %u, %u", msg.gain,
                         msg.white_balance_blue, msg.white_balance_red);
      gain_ = msg.gain;

      spinnaker_.setGain(static_cast<float>(gain_));
      wb_blue_ = msg.white_balance_blue;
      wb_red_ = msg.white_balance_red;

      // TODO(mhosmar):
      // spinnaker_.setBRWhiteBalance(false, wb_blue_, wb_red_);
    }
    catch (std::runtime_error& e)
    {
      NODELET_ERROR("gainWBCallback failed with error: %s", e.what());
    }
  }

  /* Class Fields */
  std::shared_ptr<dynamic_reconfigure::Server<spinnaker_camera_driver::SpinnakerConfig> > srv_;  ///< Needed to
                                                                                                 ///  initialize
                                                                                                 ///  and keep the
  /// dynamic_reconfigure::Server
  /// in scope.
  std::shared_ptr<image_transport::ImageTransport> it_;  ///< Needed to initialize and keep the ImageTransport in
                                                         /// scope.
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;  ///< Needed to initialize and keep the
                                                                   /// CameraInfoManager in scope.
  image_transport::CameraPublisher it_pub_;                        ///< CameraInfoManager ROS publisher
  std::shared_ptr<diagnostic_updater::DiagnosedPublisher<wfov_camera_msgs::WFOVImage> > pub_;  ///< Diagnosed
  std::shared_ptr<ros::Publisher> diagnostics_pub_;
  /// publisher, has to be
  /// a pointer because of
  /// constructor
  /// requirements
  ros::Subscriber sub_;  ///< Subscriber for gain and white balance changes.

  std::mutex connect_mutex_;

  diagnostic_updater::Updater updater_;  ///< Handles publishing diagnostics messages.
  double min_freq_;
  double max_freq_;

  SpinnakerCamera spinnaker_;      ///< Instance of the SpinnakerCamera library, used to interface with the hardware.
  sensor_msgs::CameraInfoPtr ci_;  ///< Camera Info message.
  std::string frame_id_;           ///< Frame id for the camera messages, defaults to 'camera'
  std::shared_ptr<boost::thread> pubThread_;  ///< The thread that reads and publishes the images.
  std::shared_ptr<boost::thread> diagThread_;  ///< The thread that reads and publishes the diagnostics.

  std::unique_ptr<DiagnosticsManager> diag_man;

  double gain_;
  uint16_t wb_blue_;
  uint16_t wb_red_;

  // Parameters for cameraInfo
  size_t binning_x_;     ///< Camera Info pixel binning along the image x axis.
  size_t binning_y_;     ///< Camera Info pixel binning along the image y axis.
  size_t roi_x_offset_;  ///< Camera Info ROI x offset
  size_t roi_y_offset_;  ///< Camera Info ROI y offset
  size_t roi_height_;    ///< Camera Info ROI height
  size_t roi_width_;     ///< Camera Info ROI width
  bool do_rectify_;  ///< Whether or not to rectify as if part of an image.  Set to false if whole image, and true if in
                     /// ROI mode.

  // For GigE cameras:
  /// If true, GigE packet size is automatically determined, otherwise packet_size_ is used:
  bool auto_packet_size_;
  /// GigE packet size:
  int packet_size_;
  /// GigE packet delay:
  int packet_delay_;

  /// Configuration:
  spinnaker_camera_driver::SpinnakerConfig config_;
};

PLUGINLIB_EXPORT_CLASS(spinnaker_camera_driver::SpinnakerCameraNodelet,
                       nodelet::Nodelet)  // Needed for Nodelet declaration
}  // namespace spinnaker_camera_driver
