// -*-c++-*--------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef SPINNAKER_CAMERA_DRIVER__CAMERA_HPP_
#define SPINNAKER_CAMERA_DRIVER__CAMERA_HPP_

#include <camera_info_manager/camera_info_manager.hpp>
#include <deque>
#include <flir_camera_msgs/msg/camera_control.hpp>
#include <flir_camera_msgs/msg/image_meta_data.hpp>
#include <image_transport/image_transport.hpp>
#include <limits>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <spinnaker_camera_driver/image.hpp>
#include <spinnaker_camera_driver/spinnaker_wrapper.hpp>
#include <spinnaker_camera_driver/synchronizer.hpp>
#include <std_msgs/msg/float64.hpp>
#include <thread>

namespace spinnaker_camera_driver
{
class ExposureController;  // forward decl
class Camera
{
public:
  using ImageConstPtr = spinnaker_camera_driver::ImageConstPtr;
  explicit Camera(
    rclcpp::Node * node, image_transport::ImageTransport * it, const std::string & prefix,
    bool useStatus = true);
  ~Camera();

  bool start();
  bool stop();
  void setSynchronizer(const std::shared_ptr<Synchronizer> & s) { synchronizer_ = s; }
  void setExposureController(const std::shared_ptr<ExposureController> & e)
  {
    exposureController_ = e;
  }
  const std::string & getName() const { return (name_); }
  const std::string & getPrefix() const { return (prefix_); }

private:
  struct NodeInfo
  {
    enum NodeType { INVALID, ENUM, FLOAT, INT, BOOL, COMMAND };
    explicit NodeInfo(const std::string & n, const std::string & nodeType);
    std::string name;
    NodeType type{INVALID};
    rcl_interfaces::msg::ParameterDescriptor descriptor;
  };
  void processImage(const ImageConstPtr & image);
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
  bool execute(const std::string & nodeName);
  bool readParameterDefinitionFile();

  rclcpp::Time getAdjustedTimeStamp(uint64_t t, int64_t sensorTime);

  void run();  // thread

  rcl_interfaces::msg::SetParametersResult parameterChanged(
    const std::vector<rclcpp::Parameter> & params);
  void controlCallback(const flir_camera_msgs::msg::CameraControl::UniquePtr msg);
  void printStatus();
  void checkSubscriptions();
  void doPublish(const ImageConstPtr & im);
  rclcpp::Logger get_logger()
  {
    return rclcpp::get_logger(
      !name_.empty() ? name_ : (serial_.empty() ? std::string("camera") : serial_));
  }

  template <class T>
  T safe_declare(const std::string & name, const T & def)
  {
    try {
      return (node_->declare_parameter<T>(name, def));
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
      T value(def);
      (void)node_->get_parameter_or<T>(name, value, def);
      return (value);
    }
  }

  void safe_declare(
    const std::string & name, const rclcpp::ParameterValue & pv,
    const rcl_interfaces::msg::ParameterDescriptor & desc)
  {
    try {
      node_->declare_parameter(name, pv, desc, false);
    } catch (rclcpp::exceptions::InvalidParameterTypeException & e) {
      RCLCPP_WARN_STREAM(
        get_logger(), "overwriting bad param with default: " + std::string(e.what()));
      node_->declare_parameter(name, pv, desc, true);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
      // do nothing
    }
  }

  // ----- variables --
  std::string prefix_;
  std::string topicPrefix_;
  rclcpp::Node * node_;
  image_transport::ImageTransport * imageTransport_;
  image_transport::CameraPublisher pub_;
  rclcpp::Publisher<flir_camera_msgs::msg::ImageMetaData>::SharedPtr metaPub_;
  std::string serial_;
  std::string name_;
  std::string cameraInfoURL_;
  std::string frameId_;
  std::string parameterFile_;
  double frameRate_;
  double exposureTime_;  // in microseconds
  bool autoExposure_;    // if auto exposure is on/off
  bool dumpNodeMap_{false};
  bool debug_{false};
  bool quiet_{false};
  bool computeBrightness_{false};
  double acquisitionTimeout_{3.0};
  bool adjustTimeStamp_{false};
  bool useIEEE1588_{false};
  double minIEEE1588Offset_{0};
  double maxIEEE1588Offset_{0.1};
  double ptpOffset_{0};                 // in seconds
  bool connectWhileSubscribed_{false};  // if true, connects to SDK when subscription happens
  bool enableExternalControl_{false};
  uint32_t currentExposureTime_{0};
  double averageTimeDifference_{std::numeric_limits<double>::quiet_NaN()};
  int64_t baseTimeOffset_{0};
  float currentGain_{std::numeric_limits<float>::lowest()};
  std::shared_ptr<spinnaker_camera_driver::SpinnakerWrapper> wrapper_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager_;
  sensor_msgs::msg::Image imageMsg_;
  sensor_msgs::msg::CameraInfo cameraInfoMsg_;
  flir_camera_msgs::msg::ImageMetaData metaMsg_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callbackHandle_;  // keep alive callbacks
  rclcpp::TimerBase::SharedPtr statusTimer_;
  rclcpp::TimerBase::SharedPtr checkSubscriptionsTimer_;
  bool cameraRunning_{false};
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<ImageConstPtr> bufferQueue_;
  size_t maxBufferQueueSize_{4};
  std::shared_ptr<std::thread> thread_;
  bool keepRunning_{true};
  std::map<std::string, NodeInfo> parameterMap_;
  std::vector<std::string> parameterList_;  // remember original ordering
  rclcpp::Subscription<flir_camera_msgs::msg::CameraControl>::SharedPtr controlSub_;
  uint32_t publishedCount_{0};
  uint32_t droppedCount_{0};
  uint32_t queuedCount_{0};
  rclcpp::Time lastStatusTime_;
  int qosDepth_{4};
  std::shared_ptr<Synchronizer> synchronizer_;
  std::shared_ptr<ExposureController> exposureController_;
  bool firstSynchronizedFrame_{true};
};
}  // namespace spinnaker_camera_driver
#endif  // SPINNAKER_CAMERA_DRIVER__CAMERA_HPP_
