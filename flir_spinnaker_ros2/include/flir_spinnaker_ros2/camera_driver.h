// -*-c++-*--------------------------------------------------------------------
// Copyright 2020 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FLIR_SPINNAKER_ROS2__CAMERA_DRIVER_H_
#define FLIR_SPINNAKER_ROS2__CAMERA_DRIVER_H_

#include <flir_spinnaker_common/driver.h>
#include <flir_spinnaker_common/image.h>

#include <camera_control_msgs_ros2/msg/camera_control.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <deque>
#include <image_meta_msgs_ros2/msg/image_meta_data.hpp>
#include <image_transport/image_transport.hpp>
#include <limits>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <thread>

namespace flir_spinnaker_ros2
{
class CameraDriver : public rclcpp::Node
{
public:
  typedef flir_spinnaker_common::ImageConstPtr ImageConstPtr;
  explicit CameraDriver(const rclcpp::NodeOptions & options);
  ~CameraDriver();

  bool start();
  bool stop();

private:
  struct NodeInfo
  {
    enum NodeType { INVALID, ENUM, FLOAT, INT, BOOL };
    explicit NodeInfo(const std::string & n, const std::string & nodeType);
    std::string name;
    NodeType type{INVALID};
    rcl_interfaces::msg::ParameterDescriptor descriptor;
  };
  void publishImage(const ImageConstPtr & image);
  void readParameters();
  void printCameraInfo();
  void startCamera();
  bool stopCamera();
  void createCameraParameters();
  void setParameter(const NodeInfo & ni, const rclcpp::Parameter & p);
  bool setEnum(const std::string & nodeName, const std::string & v = "");
  bool setDouble(const std::string & nodeName, double v);
  bool setInt(const std::string & nodeName, int v);
  bool setBool(const std::string & nodeName, bool v);
  bool readParameterFile();
  rclcpp::Time getAdjustedTimeStamp(uint64_t t, int64_t sensorTime);

  void run();  // thread

  rcl_interfaces::msg::SetParametersResult parameterChanged(
    const std::vector<rclcpp::Parameter> & params);
  void controlCallback(
    const camera_control_msgs_ros2::msg::CameraControl::UniquePtr msg);
  void printStatus();
  void doPublish(const ImageConstPtr & im);
  // ----- variables --
  std::shared_ptr<rclcpp::Node> node_;
  image_transport::CameraPublisher pub_;
  rclcpp::Publisher<image_meta_msgs_ros2::msg::ImageMetaData>::SharedPtr
    metaPub_;
  std::string serial_;
  std::string cameraInfoURL_;
  std::string frameId_;
  std::string parameterFile_;
  double frameRate_;
  double exposureTime_;  // in microseconds
  bool autoExposure_;    // if auto exposure is on/off
  bool dumpNodeMap_{false};
  bool debug_{false};
  bool computeBrightness_{false};
  double acquisitionTimeout_{3.0};
  bool adjustTimeStamp_{false};
  uint32_t currentExposureTime_{0};
  double averageTimeDifference_{std::numeric_limits<double>::quiet_NaN()};
  int64_t baseTimeOffset_{0};
  float currentGain_{std::numeric_limits<float>::lowest()};
  std::shared_ptr<flir_spinnaker_common::Driver> driver_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager_;
  sensor_msgs::msg::Image imageMsg_;
  sensor_msgs::msg::CameraInfo cameraInfoMsg_;
  image_meta_msgs_ros2::msg::ImageMetaData metaMsg_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr
    callbackHandle_;  // keep alive callbacks
  rclcpp::TimerBase::SharedPtr statusTimer_;
  bool cameraRunning_{false};
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<ImageConstPtr> bufferQueue_;
  size_t maxBufferQueueSize_{4};
  std::shared_ptr<std::thread> thread_;
  bool keepRunning_{true};
  std::map<std::string, NodeInfo> parameterMap_;
  std::vector<std::string> parameterList_;  // remember original ordering
  rclcpp::Subscription<camera_control_msgs_ros2::msg::CameraControl>::SharedPtr
    controlSub_;
  uint32_t publishedCount_{0};
  uint32_t droppedCount_{0};
  uint32_t queuedCount_{0};
  rclcpp::Time lastStatusTime_;
  int qosDepth_{4};
};
}  // namespace flir_spinnaker_ros2
#endif  // FLIR_SPINNAKER_ROS2__CAMERA_DRIVER_H_
