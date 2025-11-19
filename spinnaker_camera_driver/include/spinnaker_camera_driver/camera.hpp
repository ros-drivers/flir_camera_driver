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
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <flir_camera_msgs/msg/camera_control.hpp>
#include <flir_camera_msgs/msg/image_meta_data.hpp>
#include <image_transport/image_transport.hpp>
#include <limits>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <spinnaker_camera_driver/diagnostic_levels.hpp>
#include <spinnaker_camera_driver/image.hpp>
#include <spinnaker_camera_driver/lifecycle_types.hpp>
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
  using CompositeDiagnosticsTask = diagnostic_updater::CompositeDiagnosticTask;
  using FunctionDiagnosticsTask = diagnostic_updater::FunctionDiagnosticTask;
  using DiagnosticStatusWrapper = diagnostic_updater::DiagnosticStatusWrapper;
  /*
    The Constructor takes a long list of interfaces to support older distros like Humble
   */
  explicit Camera(
    const std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> &,
    const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> &,
    const std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> &,
    const std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> &,
    const std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> &,
    const std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> &,
    const std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> &,
    image_transport::ImageTransport * it, camera_info_manager::CameraInfoManager * im,
    const std::string & logName, const std::string & prefix, bool useStatus = true);
  ~Camera();

  void setSynchronizer(const std::shared_ptr<Synchronizer> & s) { synchronizer_ = s; }
  void setExposureController(const std::shared_ptr<ExposureController> & e)
  {
    exposureController_ = e;
  }
  const std::string & getName() const { return (name_); }
  const std::string & getPrefix() const { return (prefix_); }

  bool configure();    // corresponds to lifecycle configure
  bool activate();     // corresponds to lifecycle activate
  bool deactivate();   // corresponds to lifecycle deactivate
  bool deconfigure();  // corresponds to lifecycle cleanup

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
  bool startStreaming();
  bool stopStreaming();
  void createCameraParameters();
  void setParameter(const NodeInfo & ni, const rclcpp::Parameter & p);
  bool setEnum(const std::string & nodeName, const std::string & v = "");
  bool setDouble(const std::string & nodeName, double v);
  bool setInt(const std::string & nodeName, int v);
  bool setBool(const std::string & nodeName, bool v);
  bool execute(const std::string & nodeName);
  bool readParameterDefinitionFile();
  void startDiagnostics();
  void stopDiagnostics();
  void makePublishers();
  void destroyPublishers();
  void makeSubscribers();
  void destroySubscribers();
  void startTimers();
  void stopTimers();
  void openDevice();
  void closeDevice();

  rclcpp::Time getAdjustedTimeStamp(uint64_t t, int64_t sensorTime);

  void run();  // thread

  rcl_interfaces::msg::SetParametersResult parameterChanged(
    const std::vector<rclcpp::Parameter> & params);
  void controlCallback(const flir_camera_msgs::msg::CameraControl::UniquePtr msg);
  void updateStatus();
  void checkSubscriptions();
  void doPublish(const ImageConstPtr & im);
  rclcpp::Logger get_logger() { return (rclcpp::get_logger(logName_)); }

  template <class T>
  T safe_declare(const std::string & name, const T & def)
  {
    try {
      return (
        node_parameters_interface_->declare_parameter(name, rclcpp::ParameterValue(def)).get<T>());
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
      const rclcpp::Parameter p = node_parameters_interface_->get_parameter(name);
      return (p.get_parameter_value().get<T>());
    }
  }
  template <class T>
  T safePrefixedDeclare(const std::string & name, const T & def)
  {
    return (safe_declare<T>(prefix_ + name, def));
  }

  auto clock() { return node_clock_interface_->get_clock(); }
  auto name() { return node_base_interface_->get_name(); }

  void safe_declare(
    const std::string & name, const rclcpp::ParameterValue & pv,
    const rcl_interfaces::msg::ParameterDescriptor & desc)
  {
    try {
      node_parameters_interface_->declare_parameter(name, pv, desc, false);
    } catch (rclcpp::exceptions::InvalidParameterTypeException & e) {
      RCLCPP_WARN_STREAM(
        get_logger(), "overwriting bad param with default: " + std::string(e.what()));
      node_parameters_interface_->declare_parameter(name, pv, desc, true);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
      // do nothing
    }
  }
  void updateDiagnosticsStatus(
    DiagnosticStatusWrapper & status, const std::string & label, const int val,
    const DiagnosticLevels<int> & levels);
  void incompleteDiagnostics(DiagnosticStatusWrapper & status);
  void droppedDiagnostics(DiagnosticStatusWrapper & status);
  void acquisitionDiagnostics(DiagnosticStatusWrapper & status);

  // ----- variables --
  std::string prefix_;
  std::string topicPrefix_;
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface_;
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_parameters_interface_;
  std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface_;
  std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface_;
  std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface_;
  std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface_;
  std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface_;
  image_transport::ImageTransport * imageTransport_{nullptr};
  camera_info_manager::CameraInfoManager * infoManager_{nullptr};
  image_transport::CameraPublisher pub_;
  rclcpp::Publisher<flir_camera_msgs::msg::ImageMetaData>::SharedPtr metaPub_;
  std::string serial_;
  std::string logName_;
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
  double ptpOffset_{0};                    // in seconds
  bool streamOnlyWhileSubscribed_{false};  // if true, streams from camera only while subscribed
  bool enableExternalControl_{false};
  uint32_t currentExposureTime_{0};
  double averageTimeDifference_{std::numeric_limits<double>::quiet_NaN()};
  int64_t baseTimeOffset_{0};
  float currentGain_{std::numeric_limits<float>::lowest()};
  std::shared_ptr<spinnaker_camera_driver::SpinnakerWrapper> wrapper_;
  sensor_msgs::msg::Image imageMsg_;
  sensor_msgs::msg::CameraInfo cameraInfoMsg_;
  flir_camera_msgs::msg::ImageMetaData metaMsg_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callbackHandle_;  // keep alive callbacks
  rclcpp::TimerBase::SharedPtr statusTimer_;
  rclcpp::TimerBase::SharedPtr checkSubscriptionsTimer_;
  bool cameraStreaming_{false};
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
  bool runStatusTimer_{false};
  // --------- related to diagnostics
  std::shared_ptr<diagnostic_updater::Updater> updater_;
  std::shared_ptr<diagnostic_updater::TopicDiagnostic> topicDiagnostic_;
  std::shared_ptr<diagnostic_updater::FrequencyStatus> imageArrivalDiagnostic_;
  double maxFreqDiag_{0};
  double minFreqDiag_{0};
  CompositeDiagnosticsTask imageTask_;
  FunctionDiagnosticsTask incompleteFrameTask_;
  uint64_t numIncompletes_{0};
  DiagnosticLevels<int> incompleteLevels_;
  FunctionDiagnosticsTask dropFrameTask_;
  DiagnosticLevels<int> dropLevels_;
  uint64_t lastFrameId_{0};
  uint64_t numDrops_{0};
  FunctionDiagnosticsTask acquisitionErrorTask_;
  uint64_t acquisitionTimeouts_{0};
  bool acquisitionError_{false};
};
}  // namespace spinnaker_camera_driver
#endif  // SPINNAKER_CAMERA_DRIVER__CAMERA_HPP_
