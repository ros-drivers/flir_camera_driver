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
#include <camera_info_manager/camera_info_manager.h>  // ROS library that publishes CameraInfo topics
#include <diagnostic_updater/diagnostic_updater.h>    // Headers for publishing diagnostic messages.
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>            // Needed for the dynamic_reconfigure gui service to run
#include <image_exposure_msgs/ExposureSequence.h>  // Message type for configuring gain and white balance.
#include <image_transport/image_transport.h>       // ROS library that allows sending compressed images
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/CameraInfo.h>  // ROS message header for CameraInfo
#include <wfov_camera_msgs/WFOVImage.h>

#include <boost/thread.hpp>  // Needed for the nodelet to launch the reading thread.
#include <csignal>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "ros/ros.h"
#include "spinnaker_camera_driver/SpinnakerCamera.h"  // The actual standalone library for the Spinnakers
#include "spinnaker_camera_driver/diagnostics.h"

namespace spinnaker_camera_driver
{
class SpinnakerStereoCameraNodelet : public nodelet::Nodelet
{
public:
  SpinnakerStereoCameraNodelet() : nodelet::Nodelet()
  {
  }

  ~SpinnakerStereoCameraNodelet()
  {
    ROS_WARN_STREAM("Stopping camera capture.");

    std::lock_guard<std::mutex> scopedLock(connect_mutex_);

    for (size_t i = 0; i < camera_ids_.size(); i++)
    {
      uint32_t camera_id = camera_ids_[i];
      if (pubThreads_[i])
      {
        pubThreads_[i]->interrupt();
        pubThreads_[i]->join();

        try
        {
          NODELET_DEBUG_ONCE("Stopping camera capture.");
          spinnaker_cameras_.at(camera_id)->stop();
          NODELET_DEBUG_ONCE("Disconnecting from camera.");
          spinnaker_cameras_.at(camera_id)->disconnect();
        }
        catch (const std::runtime_error& e)
        {
          NODELET_ERROR("%s", e.what());
        }
      }
    }
  }

private:
  /*!
   * \brief Function that allows reconfiguration of the camera.
   *
   * This function serves as a callback for the dynamic reconfigure service.  It simply passes the configuration
   * object to the driver to allow the camera to reconfigure. \param config  camera_library::CameraConfig object
   * passed by reference.  Values will be changed to those the driver is currently using. \param level driver_base
   * reconfiguration level.  See driver_base/SensorLevels.h for more information.
   */

  void paramCallback(const spinnaker_camera_driver::SpinnakerConfig& config, uint32_t level)
  {
    config_ = config;

    try
    {
      NODELET_INFO("Dynamic reconfigure callback with level: %u", level);

      for (auto camera_id_pair : spinnaker_cameras_)
      {
        camera_id_pair.second->setNewConfiguration(config, level);
      }

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
          (config.image_format_roi_width < spinnaker_cameras_.at(camera_ids_.at(0))->getWidthMax() ||
           config.image_format_roi_height < spinnaker_cameras_.at(camera_ids_.at(0))->getHeightMax()))
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

  /*!
   * \brief Connection callback to only do work when someone is listening.
   *
   * This function will connect/disconnect from the camera depending on who is using the output.
   */
  void connectCb()
  {
    pubThreads_.resize(camera_ids_.size());
    // We need to connect
    for (size_t i = 0; i < camera_ids_.size(); i++)
    {
      if (!pubThreads_[i])
      {
        // Start the thread to loop through and publish messages
        pubThreads_[i].reset(new boost::thread(
            boost::bind(&spinnaker_camera_driver::SpinnakerStereoCameraNodelet::devicePoll, this, camera_ids_[i])));
      }
    }
  }

  /*!
   * \brief Serves as a psuedo constructor for nodelets.
   *
   * This function needs to do the MINIMUM amount of work to get the nodelet running.  Nodelets should not call
   * blocking functions here.
   */
  void onInit()
  {
    // Get nodeHandles
    ros::NodeHandle& nh = getMTNodeHandle();
    ros::NodeHandle& pnh = getMTPrivateNodeHandle();

    // Get a serial numbers through ros
    uint32_t primary_cam_id = getCameraSerial("primary_cam_serial", pnh);
    uint32_t secondary_cam_id = getCameraSerial("secondary_cam_serial", pnh);

    if (primary_cam_id == 0)
    {
      NODELET_ERROR_STREAM("Could not find primary_camera_serial. Need to provide id for primary camera.");
      ros::requestShutdown();
      ros::shutdown();
      std::raise(SIGINT);
      std::raise(SIGABRT);
      exit(-1);
    }

    if (secondary_cam_id == 0)
    {
      NODELET_ERROR_STREAM("Could not find secondary_camera_serial. Need to provide id for secondary camera.");
      ros::requestShutdown();
      ros::shutdown();
      std::raise(SIGINT);
      std::raise(SIGABRT);
      exit(-1);
    }

    camera_ids_.emplace_back(primary_cam_id);
    camera_ids_.emplace_back(secondary_cam_id);

    NODELET_INFO_STREAM("Primary camera serial: " << primary_cam_id);
    NODELET_INFO_STREAM("Secondary camera serial: " << secondary_cam_id);

    if (spinnaker_cameras_.find(primary_cam_id) == spinnaker_cameras_.end())
    {
      spinnaker_cameras_[primary_cam_id] = std::make_shared<spinnaker_camera_driver::SpinnakerCamera>();
    }
    spinnaker_cameras_.at(primary_cam_id)->setDesiredCamera(primary_cam_id);
    spinnaker_cameras_.at(primary_cam_id)->setCameraMode("primary");

    if (spinnaker_cameras_.find(secondary_cam_id) == spinnaker_cameras_.end())
    {
      spinnaker_cameras_[secondary_cam_id] = std::make_shared<spinnaker_camera_driver::SpinnakerCamera>();
    }
    spinnaker_cameras_.at(secondary_cam_id)->setDesiredCamera(secondary_cam_id);
    spinnaker_cameras_.at(secondary_cam_id)->setCameraMode("secondary");

    // Get GigE camera parameters:
    pnh.param<int>("packet_size", packet_size_, 1400);
    pnh.param<bool>("auto_packet_size", auto_packet_size_, true);
    pnh.param<int>("packet_delay", packet_delay_, 4000);

    // TODO(mhosmar):  Set GigE parameters:
    // spinnaker_.setGigEParameters(auto_packet_size_, packet_size_, packet_delay_);

    // Get the location of our camera config yaml
    std::string primary_camera_info_url, secondary_camera_info_url;
    pnh.param<std::string>("primary_camera_info_url", primary_camera_info_url, "");
    pnh.param<std::string>("secondary_camera_info_url", secondary_camera_info_url, "");

    // Get the desired frame_id, set to 'camera' if not found
    std::string primary_camera_frame, secondary_camera_frame;
    pnh.param<std::string>("primary_camera_frame", primary_camera_frame, "left");
    pnh.param<std::string>("secondary_camera_frame", secondary_camera_frame, "right");

    camera_frames_[primary_cam_id] = primary_camera_frame;
    camera_frames_[secondary_cam_id] = secondary_camera_frame;

    // Do not call the connectCb function until after we are done initializing.
    std::lock_guard<std::mutex> scopedLock(connect_mutex_);

    // Start up the dynamic_reconfigure service, note that this needs to stick around after this function ends
    srv_ = std::make_shared<dynamic_reconfigure::Server<spinnaker_camera_driver::SpinnakerConfig>>(pnh);
    dynamic_reconfigure::Server<spinnaker_camera_driver::SpinnakerConfig>::CallbackType f =
        boost::bind(&spinnaker_camera_driver::SpinnakerStereoCameraNodelet::paramCallback, this, _1, _2);

    srv_->setCallback(f);

    // queue size of ros publisher
    int queue_size;
    pnh.param<int>("queue_size", queue_size, 5);

    // Start the camera info manager and attempt to load any configurations
    camera_info_managers_[primary_cam_id] = std::make_shared<camera_info_manager::CameraInfoManager>(
        ros::NodeHandle(nh, primary_camera_frame), std::to_string(primary_cam_id), primary_camera_info_url);
    camera_info_managers_[secondary_cam_id] = std::make_shared<camera_info_manager::CameraInfoManager>(
        ros::NodeHandle(nh, secondary_camera_frame), std::to_string(secondary_cam_id), secondary_camera_info_url);

    // Publish topics using ImageTransport through camera_info_manager (gives cool things like compression)
    img_transport_.reset(new image_transport::ImageTransport(nh));

    // For both image transport, same callback as we will make sure the cameras are initialized in correct order
    image_transport::SubscriberStatusCallback cb = boost::bind(&SpinnakerStereoCameraNodelet::connectCb, this);
    image_transport_pubs_[primary_cam_id] =
        img_transport_->advertiseCamera(primary_camera_frame + "/image_raw", queue_size, cb);
    image_transport_pubs_[secondary_cam_id] =
        img_transport_->advertiseCamera(secondary_camera_frame + "/image_raw", queue_size, cb);

    primary_camera_config_set_ = false;
  }

  /*!
   * \brief Function for the boost::thread to grabImages and publish them.
   *
   * This function continues until the thread is interupted.  Responsible for getting sensor_msgs::Image and
   * publishing them.
   */
  void devicePoll(uint32_t camera_id)
  {
    ROS_WARN_STREAM("devicePoll for camera_id: " << camera_id);

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

      std::shared_ptr<SpinnakerCamera> camera = spinnaker_cameras_.at(camera_id);
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
            NODELET_WARN("Disconnecting from camera.");
            camera->disconnect();
            NODELET_WARN("Disconnected from camera.");

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
            NODELET_WARN_STREAM("Connecting to camera: " << camera_id);
            camera->connect();
            NODELET_WARN("Connected to camera.");

            // Setting camera as primary or secondary or auto mode
            NODELET_WARN_STREAM("Setting configuration for camera " << camera_id);
            camera->updateCameraMode();
            // Set last configuration, forcing the reconfigure level to stop
            camera->setNewConfiguration(config_, SpinnakerCamera::LEVEL_RECONFIGURE_STOP);

            if (camera->getMode() == "secondary")
            {
              while (!primary_camera_config_set_)
              {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
              }
              camera->setMutipleCameraSynchronization();
            }
            else if (camera->getMode() == "primary")
            {
              camera->setMutipleCameraSynchronization();
              primary_camera_config_set_ = true;
            }

            NODELET_WARN_STREAM("Done setting configuration ");

            // Set the timeout for grabbing images.
            try
            {
              double timeout;
              getMTPrivateNodeHandle().param("timeout", timeout, 1.0);

              NODELET_WARN_STREAM("Setting timeout to: " << timeout << " for camera " << camera_id);
              camera->setTimeout(timeout);
              NODELET_WARN_STREAM("Done setting timeout for camera " << camera_id);
            }
            catch (const std::runtime_error& e)
            {
              NODELET_ERROR("%s", e.what());
            }

            // Subscribe to gain and white balance changes
            {
              std::lock_guard<std::mutex> scopedLock(connect_mutex_);
              sub_ = getMTNodeHandle().subscribe("image_exposure_sequence", 10,
                                                 &spinnaker_camera_driver::SpinnakerStereoCameraNodelet::gainWBCallback,
                                                 this);
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
            NODELET_WARN_STREAM("Starting camera " << camera_id);
            camera->start();
            NODELET_WARN("Started camera.");

            NODELET_DEBUG(
                "Attention: if nothing subscribes to the camera topic, the camera_info is not published "
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
            // Set the CameraInfo message

            sensor_msgs::Image image;
            ros::Time stamp;
            sensor_msgs::CameraInfoPtr camera_info;
            std::string cam_frame;

            cam_frame = camera_frames_.at(camera_id);
            // Get the image from the camera library
            if (camera->getMode() == "primary")
            {
              camera->grabImage(&image, cam_frame, stamp);
              current_stamp_ = stamp.toNSec();
              primary_camera_capture_finished_ = true;
            }
            else if (camera->getMode() == "secondary")
            {
              while (!primary_camera_capture_finished_)
              {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
              }
              camera->grabImage(&image, cam_frame, stamp);
              primary_camera_capture_finished_ = false;
            }

            camera_info.reset(new sensor_msgs::CameraInfo(camera_info_managers_.at(camera_id)->getCameraInfo()));

            stamp = ros::Time(current_stamp_ * 1e-9, current_stamp_ % 1000000000);
            camera_info->header.stamp = stamp;
            camera_info->header.frame_id = cam_frame;
            // The height, width, distortion model, and parameters are all filled in by camera info manager.
            camera_info->binning_x = binning_x_;
            camera_info->binning_y = binning_y_;
            camera_info->roi.x_offset = roi_x_offset_;
            camera_info->roi.y_offset = roi_y_offset_;
            camera_info->roi.height = roi_height_;
            camera_info->roi.width = roi_width_;
            camera_info->roi.do_rectify = do_rectify_;

            // Publish the message using standard image transport
            if (image_transport_pubs_.at(camera_id).getNumSubscribers() > 0)
            {
              sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image(image));
              image_msg->header.stamp = stamp;
              image_msg->header.frame_id = camera_frames_.at(camera_id);
              image_transport_pubs_.at(camera_id).publish(image_msg, camera_info);
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
      // updater_.update();
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

      for (uint32_t camera_id : camera_ids_)
      {
        spinnaker_cameras_.at(camera_id)->setGain(static_cast<float>(gain_));
      }
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

  uint32_t getCameraSerial(const std::string& ros_param, ros::NodeHandle& pnh)
  {
    int serial = 0;
    XmlRpc::XmlRpcValue serial_xmlrpc;
    pnh.getParam(ros_param, serial_xmlrpc);
    if (serial_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      pnh.param<int>(ros_param, serial, 0);
    }
    else if (serial_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      std::string serial_str;
      pnh.param<std::string>(ros_param, serial_str, "0");
      std::istringstream(serial_str) >> serial;
    }
    else
    {
      NODELET_WARN_ONCE("Serial XMLRPC type.");
      serial = 0;
    }
    return (uint32_t)serial;
  }

  /* Class Fields */

  std::vector<std::uint32_t> camera_ids_;

  // Needed to initialize and keep the dynamic_reconfigure::Server in scope.
  std::shared_ptr<dynamic_reconfigure::Server<spinnaker_camera_driver::SpinnakerConfig>> srv_;

  // Using hashmap of objects for mutiple cameras by serial.
  // This would be better we someone extends this to multiple secondary cameras
  // on the same board than using just a vector.

  ///< Needed to initialize and keep the ImageTransport in scope.
  std::shared_ptr<image_transport::ImageTransport> img_transport_;

  ///< Needed to initialize and keep the CameraInfoManager in scope.
  std::unordered_map<uint32_t, std::shared_ptr<camera_info_manager::CameraInfoManager>> camera_info_managers_;

  ///< CameraInfoManager ROS publisher
  std::unordered_map<uint32_t, image_transport::CameraPublisher> image_transport_pubs_;

  ///< Subscriber for gain and white balance changes.
  ///< One common subscriber as we want to change gain and white balance of all cameras.
  ros::Subscriber sub_;

  std::mutex connect_mutex_;

  ///< Instance of the SpinnakerCamera library, used to interface with the hardware.
  std::unordered_map<uint32_t, std::shared_ptr<SpinnakerCamera>> spinnaker_cameras_;

  ///< Frame id for the camera messages, defaults to serial ids
  std::unordered_map<uint32_t, std::string> camera_frames_;

  std::vector<std::shared_ptr<boost::thread>> pubThreads_;  ///< The thread that reads and publishes the images.
  std::shared_ptr<boost::thread> diagThread_;               ///< The thread that reads and publishes the diagnostics.

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
  bool do_rectify_;      ///< Whether or not to rectify as if part of an image.  Set to false if whole image, and true
                         ///< if in
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

  std::atomic<bool> primary_camera_config_set_;
  std::atomic<uint64_t> current_stamp_;
  std::atomic<bool> primary_camera_capture_finished_;
};

// Needed for Nodelet declaration
PLUGINLIB_EXPORT_CLASS(spinnaker_camera_driver::SpinnakerStereoCameraNodelet, nodelet::Nodelet)
}  // namespace spinnaker_camera_driver
