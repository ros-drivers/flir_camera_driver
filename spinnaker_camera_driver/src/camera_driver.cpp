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

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <image_transport/image_transport.hpp>
#include <iomanip>
#include <iostream>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <spinnaker_camera_driver/camera_driver.hpp>
#include <type_traits>

namespace spinnaker_camera_driver
{
namespace chrono = std::chrono;
//
// this complicated code is to detect an interface change
// between foxy and galactic
// See  https://stackoverflow.com/questions/1005476/
//  how-to-detect-whether-there-is-a-specific-member-variable-in-class

template <typename T, typename = bool>
struct DescSetter
{
  // don't set by default (foxy)
  static void set_dynamic_typing(T *) {}
};

template <typename T>
struct DescSetter<T, decltype((void)T::dynamic_typing, true)>
{
  // set if dynamic_typing is present
  static void set_dynamic_typing(T * desc) { desc->dynamic_typing = true; }
};

static rcl_interfaces::msg::ParameterDescriptor make_desc(const std::string name, int type)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.name = name;
  desc.type = type;
  desc.description = name;
  DescSetter<rcl_interfaces::msg::ParameterDescriptor>::set_dynamic_typing(&desc);
  return (desc);
}

static std::pair<bool, double> get_double_int_param(const rclcpp::Parameter & p)
{
  std::pair<bool, double> bd(false, 0);
  if (p.get_type() == rclcpp::PARAMETER_DOUBLE) {
    bd.second = p.as_double();
    bd.first = true;
  }
  if (p.get_type() == rclcpp::PARAMETER_INTEGER) {
    bd.second = static_cast<double>(p.as_int());
    bd.first = true;
  }
  return (bd);
}

static std::pair<bool, bool> get_bool_int_param(const rclcpp::Parameter & p)
{
  std::pair<bool, bool> bb(false, false);
  if (p.get_type() == rclcpp::PARAMETER_BOOL) {
    bb.second = p.as_bool();
    bb.first = true;
  }
  if (p.get_type() == rclcpp::PARAMETER_INTEGER) {
    bb.second = static_cast<bool>(p.as_int());
    bb.first = true;
  }
  return (bb);
}

CameraDriver::NodeInfo::NodeInfo(const std::string & n, const std::string & nodeType) : name(n)
{
  if (nodeType == "float") {
    type = FLOAT;
    descriptor = make_desc(n, rclcpp::ParameterType::PARAMETER_DOUBLE);
  } else if (nodeType == "int") {
    type = INT;
    descriptor = make_desc(n, rclcpp::ParameterType::PARAMETER_INTEGER);
  } else if (nodeType == "bool") {
    type = BOOL;
    descriptor = make_desc(n, rclcpp::ParameterType::PARAMETER_BOOL);
  } else if (nodeType == "enum") {
    type = ENUM;
    descriptor = make_desc(n, rclcpp::ParameterType::PARAMETER_STRING);
  }
}

CameraDriver::CameraDriver(const rclcpp::NodeOptions & options) : Node("cam_sync", options)
{
  lastStatusTime_ = now();
  statusTimer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration(5, 0), std::bind(&CameraDriver::printStatus, this));
  bool status = start();
  if (!status) {
    RCLCPP_ERROR(get_logger(), "startup failed!");
    throw std::runtime_error("startup of CameraDriver node failed!");
  }
}

CameraDriver::~CameraDriver()
{
  stop();
  wrapper_.reset();  // invoke destructor
}

bool CameraDriver::stop()
{
  stopCamera();
  if (wrapper_) {
    wrapper_->deInitCamera();
  }
  if (!statusTimer_->is_canceled()) {
    statusTimer_->cancel();
  }
  keepRunning_ = false;
  if (thread_) {
    thread_->join();
    thread_ = 0;
  }
  return (true);
}

bool CameraDriver::stopCamera()
{
  if (cameraRunning_ && wrapper_) {
    cameraRunning_ = false;
    return wrapper_->stopCamera();
  }
  return false;
}

void CameraDriver::printStatus()
{
  if (wrapper_) {
    const double dropRate =
      (queuedCount_ > 0) ? (static_cast<double>(droppedCount_) / static_cast<double>(queuedCount_))
                         : 0;
    const rclcpp::Time t = now();
    const rclcpp::Duration dt = t - lastStatusTime_;
    double dtns = std::max(dt.nanoseconds(), (int64_t)1);
    double outRate = publishedCount_ * 1e9 / dtns;
    RCLCPP_INFO(
      this->get_logger(), "rate [Hz] in %6.2f out %6.2f drop %3.0f%%",
      wrapper_->getReceiveFrameRate(), outRate, dropRate * 100);
    lastStatusTime_ = t;
    droppedCount_ = 0;
    publishedCount_ = 0;
    queuedCount_ = 0;
  } else {
    RCLCPP_WARN_STREAM(get_logger(), "camera " << serial_ << " is not online!");
  }
}

void CameraDriver::readParameters()
{
  serial_ = this->declare_parameter<std::string>("serial_number", "missing_serial_number");
  try {
    debug_ = this->declare_parameter(
      "debug", false, make_desc("debug", rclcpp::ParameterType::PARAMETER_BOOL));
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    RCLCPP_WARN_STREAM(get_logger(), "bad debug param type: " << e.what());
    debug_ = false;
  }
  // Flag to adjust ROS time stamps
  adjustTimeStamp_ = this->declare_parameter<bool>("adjust_timestamp", false);
  RCLCPP_INFO_STREAM(get_logger(), (adjustTimeStamp_ ? "" : "not ") << "adjusting time stamps!");

  cameraInfoURL_ = this->declare_parameter<std::string>("camerainfo_url", "");
  frameId_ = this->declare_parameter<std::string>("frame_id", get_name());
  dumpNodeMap_ = this->declare_parameter<bool>("dump_node_map", false);
  qosDepth_ = this->declare_parameter<int>("image_queue_size", 4);
  maxBufferQueueSize_ = static_cast<size_t>(this->declare_parameter<int>("buffer_queue_size", 4));
  computeBrightness_ = this->declare_parameter<bool>("compute_brightness", false);
  acquisitionTimeout_ = this->declare_parameter<double>("acquisition_timeout", 3.0);
  parameterFile_ = this->declare_parameter<std::string>("parameter_file", "parameters.cfg");
  RCLCPP_INFO_STREAM(get_logger(), "looking for serial number: " << serial_);
  callbackHandle_ = this->add_on_set_parameters_callback(
    std::bind(&CameraDriver::parameterChanged, this, std::placeholders::_1));
}

bool CameraDriver::readParameterDefinitionFile()
{
  RCLCPP_INFO_STREAM(get_logger(), "parameter definitions file: " << parameterFile_);
  YAML::Node yamlFile = YAML::LoadFile(parameterFile_);
  if (yamlFile.IsNull()) {
    RCLCPP_ERROR_STREAM(get_logger(), "cannot open file: " << parameterFile_);
    return (false);
  }
  if (!yamlFile["parameters"].IsSequence()) {
    RCLCPP_ERROR_STREAM(get_logger(), "parameter definitions lists no parameters!");
    return (false);
  }
  YAML::Node params = yamlFile["parameters"];
  for (const auto & p : params) {
    if (!p["name"]) {
      RCLCPP_WARN_STREAM(get_logger(), "ignoring parameter missing name: " << p);
      continue;
    }
    if (!p["type"]) {
      RCLCPP_WARN_STREAM(get_logger(), "ignoring parameter missing type: " << p);
      continue;
    }
    if (!p["node"]) {
      RCLCPP_WARN_STREAM(get_logger(), "ignoring parameter missing node: " << p);
      continue;
    }
    const std::string pname = p["name"].as<std::string>();
    parameterMap_.insert(
      {pname, NodeInfo(p["node"].as<std::string>(), p["type"].as<std::string>())});
    parameterList_.push_back(pname);
  }
  return (true);
}

void CameraDriver::createCameraParameters()
{
  for (const auto & name : parameterList_) {
    const auto it = parameterMap_.find(name);
    if (it != parameterMap_.end()) {
      const auto & ni = it->second;  // should always succeed
      try {
        this->declare_parameter(name, rclcpp::ParameterValue(), ni.descriptor, false);
      } catch (rclcpp::exceptions::InvalidParameterTypeException & e) {
        RCLCPP_WARN_STREAM(
          get_logger(), "overwriting bad param with default: " + std::string(e.what()));
        this->declare_parameter(name, rclcpp::ParameterValue(), ni.descriptor, true);
      }
    }
  }
}

bool CameraDriver::setEnum(const std::string & nodeName, const std::string & v)
{
  RCLCPP_INFO_STREAM(get_logger(), "setting " << nodeName << " to: " << v);
  std::string retV;  // what actually was set
  std::string msg = wrapper_->setEnum(nodeName, v, &retV);
  bool status(true);
  if (msg != "OK") {
    RCLCPP_WARN_STREAM(get_logger(), "setting " << nodeName << " failed: " << msg);
    status = false;
  }
  if (v != retV) {
    RCLCPP_WARN_STREAM(get_logger(), nodeName << " set to: " << retV << " instead of: " << v);
    status = false;
  }
  return (status);
}

bool CameraDriver::setDouble(const std::string & nodeName, double v)
{
  RCLCPP_INFO_STREAM(get_logger(), "setting " << nodeName << " to: " << v);
  double retV;  // what actually was set
  std::string msg = wrapper_->setDouble(nodeName, v, &retV);
  bool status(true);
  if (msg != "OK") {
    RCLCPP_WARN_STREAM(get_logger(), "setting " << nodeName << " failed: " << msg);
    status = false;
  }
  if (std::abs(v - retV) > 0.025 * std::abs(v + retV)) {
    RCLCPP_WARN_STREAM(get_logger(), nodeName << " set to: " << retV << " instead of: " << v);
    status = false;
  }
  return (status);
}

bool CameraDriver::setInt(const std::string & nodeName, int v)
{
  RCLCPP_INFO_STREAM(get_logger(), "setting " << nodeName << " to: " << v);
  int retV;  // what actually was set
  std::string msg = wrapper_->setInt(nodeName, v, &retV);
  bool status(true);
  if (msg != "OK") {
    RCLCPP_WARN_STREAM(get_logger(), "setting " << nodeName << " failed: " << msg);
    status = false;
  }
  if (v != retV) {
    RCLCPP_WARN_STREAM(get_logger(), nodeName << " set to: " << retV << " instead of: " << v);
    status = false;
  }
  return (status);
}

bool CameraDriver::setBool(const std::string & nodeName, bool v)
{
  RCLCPP_INFO_STREAM(get_logger(), "setting " << nodeName << " to: " << v);
  bool retV;  // what actually was set
  std::string msg = wrapper_->setBool(nodeName, v, &retV);
  bool status(true);
  if (msg != "OK") {
    RCLCPP_WARN_STREAM(get_logger(), "setting " << nodeName << " failed: " << msg);
    status = false;
  }
  if (v != retV) {
    RCLCPP_WARN_STREAM(get_logger(), nodeName << " set to: " << retV << " instead of: " << v);
    status = false;
  }
  return (status);
}

void CameraDriver::setParameter(const NodeInfo & ni, const rclcpp::Parameter & p)
{
  switch (ni.type) {
    case NodeInfo::ENUM: {
      std::string s = p.value_to_string();
      // remove quotes
      s.erase(remove(s.begin(), s.end(), '\"'), s.end());
      setEnum(ni.name, s);
      break;
    }
    case NodeInfo::FLOAT: {
      auto bd = get_double_int_param(p);
      if (bd.first) {
        setDouble(ni.name, bd.second);
      } else {
        RCLCPP_WARN_STREAM(
          get_logger(), "bad non-float " << p.get_name() << " type: " << p.get_type());
      }
      break;
    }
    case NodeInfo::INT: {
      auto bd = get_double_int_param(p);
      if (bd.first) {
        setInt(ni.name, bd.second);
      } else {
        RCLCPP_WARN_STREAM(
          get_logger(), "bad non-int " << p.get_name() << " type: " << p.get_type());
      }
      break;
    }
    case NodeInfo::BOOL: {
      auto bb = get_bool_int_param(p);
      if (bb.first) {
        setBool(ni.name, bb.second);
      } else {
        RCLCPP_WARN_STREAM(
          get_logger(), "bad non-bool " << p.get_name() << " type: " << p.get_type());
      }
      break;
    }
    default:
      RCLCPP_WARN_STREAM(get_logger(), "invalid node type in map: " << ni.type);
  }
}

rcl_interfaces::msg::SetParametersResult CameraDriver::parameterChanged(
  const std::vector<rclcpp::Parameter> & params)
{
  for (const auto & p : params) {
    const auto it = parameterMap_.find(p.get_name());
    if (it == parameterMap_.end()) {
      continue;  // ignore unknown param
    }
    if (!wrapper_) {
      RCLCPP_WARN_STREAM(get_logger(), "got parameter update while driver is not ready!");
      continue;
    }
    const NodeInfo & ni = it->second;
    if (p.get_type() == rclcpp::PARAMETER_NOT_SET) {
      continue;
    }
    try {
      setParameter(ni, p);
    } catch (const spinnaker_camera_driver::SpinnakerWrapper::Exception & e) {
      RCLCPP_WARN_STREAM(get_logger(), "param " << p.get_name() << " " << e.what());
    }
  }
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  res.reason = "all good!";
  return (res);
}

void CameraDriver::controlCallback(const flir_camera_msgs::msg::CameraControl::UniquePtr msg)
{
  /*
      RCLCPP_INFO_STREAM(get_logger(),
      "control msg: time: " << currentExposureTime_ << " -> "
      << msg->exposure_time << " gain: " << currentGain_
      << " -> " << msg->gain); */
  const uint32_t et = msg->exposure_time;
  const float gain = msg->gain;
  bool logTime(false);
  bool logGain(false);
  try {
    if (et > 0 && et != currentExposureTime_) {
      const auto it = parameterMap_.find("exposure_time");
      if (it != parameterMap_.end()) {
        const auto & ni = it->second;
        setDouble(ni.name, et);
        currentExposureTime_ = et;
        logTime = true;
      } else {
        RCLCPP_WARN_STREAM(
          get_logger(), "no node name defined for exposure_time, check .cfg file!");
      }
    }
    if (gain > std::numeric_limits<float>::lowest() && gain != currentGain_) {
      const auto it = parameterMap_.find("gain");
      if (it != parameterMap_.end()) {
        const auto & ni = it->second;
        setDouble(ni.name, gain);
        currentGain_ = gain;
        logGain = true;
      } else {
        RCLCPP_WARN_STREAM(
          get_logger(), "no node name defined for exposure_time, check .cfg file!");
      }
    }
  } catch (const spinnaker_camera_driver::SpinnakerWrapper::Exception & e) {
    RCLCPP_WARN_STREAM(get_logger(), "failed to control: " << e.what());
  }

  if (logTime) {
    RCLCPP_INFO_STREAM(get_logger(), "changed exposure time to " << et << "us");
  }
  if (logGain) {
    RCLCPP_INFO_STREAM(get_logger(), "changed gain to " << gain << "db");
  }
}

void CameraDriver::publishImage(const ImageConstPtr & im)
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    queuedCount_++;
    if (bufferQueue_.size() < maxBufferQueueSize_) {
      bufferQueue_.push_back(im);
      cv_.notify_all();
    } else {
      droppedCount_++;
    }
  }
}

void CameraDriver::run()
{
  while (keepRunning_ && rclcpp::ok()) {
    {
      ImageConstPtr img;
      {  // ------- locked section ---
        std::unique_lock<std::mutex> lock(mutex_);
        // one second timeout
        const std::chrono::microseconds timeout((int64_t)(1000000LL));
        while (bufferQueue_.empty() && keepRunning_ && rclcpp::ok()) {
          cv_.wait_for(lock, timeout);
        }
        if (!bufferQueue_.empty()) {
          img = bufferQueue_.back();
          bufferQueue_.pop_back();
        }
      }  // -------- end of locked section
      if (img && keepRunning_ && rclcpp::ok()) {
        doPublish(img);
      }
    }
  }
}

using flir_fmt = spinnaker_camera_driver::pixel_format::PixelFormat;
namespace ros_fmt = sensor_msgs::image_encodings;

static const std::unordered_map<flir_fmt, std::string> flir_2_ros{
  {{flir_fmt::INVALID, "INV"},
   {flir_fmt::Mono8, ros_fmt::MONO8},
   {flir_fmt::Mono10p, "INV"},
   {flir_fmt::Mono10Packed, "INV"},
   {flir_fmt::Mono12p, "INV"},
   {flir_fmt::Mono12Packed, "INV"},
   {flir_fmt::Mono16, ros_fmt::MONO16},
   {flir_fmt::BayerRG8, ros_fmt::BAYER_RGGB8},
   {flir_fmt::BayerRG10p, "INV"},
   {flir_fmt::BayerRG10Packed, "INV"},
   {flir_fmt::BayerRG12p, "INV"},
   {flir_fmt::BayerRG12Packed, "INV"},
   {flir_fmt::BayerRG16, ros_fmt::BAYER_RGGB16},
   {flir_fmt::BayerGR8, ros_fmt::BAYER_GRBG8},
   {flir_fmt::BayerGR16, ros_fmt::BAYER_GRBG16},
   {flir_fmt::BayerGB8, ros_fmt::BAYER_GBRG8},
   {flir_fmt::BayerGB16, ros_fmt::BAYER_GBRG16},
   {flir_fmt::BayerBG8, ros_fmt::BAYER_BGGR8},
   {flir_fmt::BayerBG16, ros_fmt::BAYER_BGGR16},
   {flir_fmt::YUV411Packed, "INV"},
   {flir_fmt::YUV422Packed, "INV"},
   {flir_fmt::YUV444Packed, "INV"},
   {flir_fmt::YCbCr8, "INV"},
   {flir_fmt::YCbCr422_8, "INV"},
   {flir_fmt::YCbCr411_8, "INV"},
   {flir_fmt::RGB8, ros_fmt::RGB8},
   {flir_fmt::RGB8Packed, ros_fmt::RGB8},
   {flir_fmt::BGR8, ros_fmt::BGR8},
   {flir_fmt::BGRa8, ros_fmt::BGRA8}}};

static std::string flir_to_ros_encoding(const flir_fmt & pf, bool * canEncode)
{
  auto it = flir_2_ros.find(pf);
  *canEncode = (it != flir_2_ros.end() && it->second != "INV") && (pf != flir_fmt::INVALID);
  return (*canEncode ? it->second : "INV");
}

// adjust ROS header stamp using camera provided meta data
rclcpp::Time CameraDriver::getAdjustedTimeStamp(uint64_t t, int64_t sensorTime)
{
  if (std::isnan(averageTimeDifference_)) {
    // capture the coarse offset between sensor and ROS time
    // at the very first time stamp.
    baseTimeOffset_ = static_cast<int64_t>(t) - sensorTime;
    averageTimeDifference_ = 0;
  }
  const double dt = (static_cast<int64_t>(t) - baseTimeOffset_ - sensorTime) * 1e-9;

  // compute exponential moving average
  constexpr double alpha = 0.01;  // average over rougly 100 samples
  averageTimeDifference_ = averageTimeDifference_ * (1.0 - alpha) + alpha * dt;

  // adjust sensor time by average difference to ROS time
  const rclcpp::Time adjustedTime = rclcpp::Time(sensorTime + baseTimeOffset_, RCL_SYSTEM_TIME) +
                                    rclcpp::Duration::from_seconds(averageTimeDifference_);
  return (adjustedTime);
}

void CameraDriver::doPublish(const ImageConstPtr & im)
{
  const auto t =
    adjustTimeStamp_ ? getAdjustedTimeStamp(im->time_, im->imageTime_) : rclcpp::Time(im->time_);
  imageMsg_.header.stamp = t;
  cameraInfoMsg_.header.stamp = t;

  bool canEncode{false};
  const std::string encoding = flir_to_ros_encoding(im->pixelFormat_, &canEncode);
  if (!canEncode) {
    RCLCPP_WARN_STREAM(
      get_logger(), "no ROS encoding for pixel format "
                      << spinnaker_camera_driver::pixel_format::to_string(im->pixelFormat_));
    return;
  }

  if (pub_.getNumSubscribers() > 0) {
    sensor_msgs::msg::CameraInfo::UniquePtr cinfo(new sensor_msgs::msg::CameraInfo(cameraInfoMsg_));
    // will make deep copy. Do we need to? Probably...
    sensor_msgs::msg::Image::UniquePtr img(new sensor_msgs::msg::Image(imageMsg_));
    bool ret =
      sensor_msgs::fillImage(*img, encoding, im->height_, im->width_, im->stride_, im->data_);
    if (!ret) {
      RCLCPP_ERROR_STREAM(get_logger(), "fill image failed!");
    } else {
      // const auto t0 = this->now();
      pub_.publish(std::move(img), std::move(cinfo));
      // const auto t1 = this->now();
      // std::cout << "dt: " << (t1 - t0).nanoseconds() * 1e-9 << std::endl;
      publishedCount_++;
    }
  }
  if (metaPub_->get_subscription_count() != 0) {
    metaMsg_.header.stamp = t;
    metaMsg_.brightness = im->brightness_;
    metaMsg_.exposure_time = im->exposureTime_;
    metaMsg_.max_exposure_time = im->maxExposureTime_;
    metaMsg_.gain = im->gain_;
    metaMsg_.camera_time = im->imageTime_;
    metaPub_->publish(metaMsg_);
  }
}

void CameraDriver::printCameraInfo()
{
  if (cameraRunning_) {
    RCLCPP_INFO_STREAM(get_logger(), "camera has pixel format: " << wrapper_->getPixelFormat());
  }
}

void CameraDriver::startCamera()
{
  if (!cameraRunning_) {
    spinnaker_camera_driver::SpinnakerWrapper::Callback cb =
      std::bind(&CameraDriver::publishImage, this, std::placeholders::_1);
    cameraRunning_ = wrapper_->startCamera(cb);
    if (!cameraRunning_) {
      RCLCPP_ERROR_STREAM(get_logger(), "failed to start camera!");
    } else {
      printCameraInfo();
    }
  }
}

bool CameraDriver::start()
{
  readParameters();
  try {
    if (!readParameterDefinitionFile()) {
      return (false);
    }
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "error reading parameter definitions: " << e.what());
    return (false);
  }

  infoManager_ =
    std::make_shared<camera_info_manager::CameraInfoManager>(this, get_name(), cameraInfoURL_);
  controlSub_ = this->create_subscription<flir_camera_msgs::msg::CameraControl>(
    "~/control", 10, std::bind(&CameraDriver::controlCallback, this, std::placeholders::_1));
  metaPub_ = create_publisher<flir_camera_msgs::msg::ImageMetaData>("~/meta", 1);

  cameraInfoMsg_ = infoManager_->getCameraInfo();
  imageMsg_.header.frame_id = frameId_;
  cameraInfoMsg_.header.frame_id = frameId_;
  metaMsg_.header.frame_id = frameId_;

  rmw_qos_profile_t qosProf = rmw_qos_profile_default;
  qosProf.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qosProf.depth = qosDepth_;  // keep at most this number of images

  qosProf.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
  qosProf.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;  // sender does not have to store
  qosProf.deadline.sec = 5;                                 // max expect time between msgs pub
  qosProf.deadline.nsec = 0;

  qosProf.lifespan.sec = 1;  // how long until msg are considered expired
  qosProf.lifespan.nsec = 0;

  qosProf.liveliness_lease_duration.sec = 10;  // time to declare client dead
  qosProf.liveliness_lease_duration.nsec = 0;

  pub_ = image_transport::create_camera_publisher(this, "~/image_raw", qosProf);
  wrapper_ = std::make_shared<spinnaker_camera_driver::SpinnakerWrapper>();
  wrapper_->setDebug(debug_);
  wrapper_->setComputeBrightness(computeBrightness_);
  wrapper_->setAcquisitionTimeout(acquisitionTimeout_);

  RCLCPP_INFO_STREAM(get_logger(), "using spinnaker lib version: " + wrapper_->getLibraryVersion());
  bool foundCamera = false;
  for (int retry = 1; retry < 6; retry++) {
    wrapper_->refreshCameraList();
    const auto camList = wrapper_->getSerialNumbers();
    if (std::find(camList.begin(), camList.end(), serial_) == camList.end()) {
      RCLCPP_WARN_STREAM(
        get_logger(), "no camera found with serial: " << serial_ << " on try # " << retry);
      for (const auto & cam : camList) {
        RCLCPP_WARN_STREAM(get_logger(), " found cameras: " << cam);
      }
      std::this_thread::sleep_for(chrono::seconds(1));
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "found camera with serial number: " << serial_);
      foundCamera = true;
      break;
    }
  }
  if (!foundCamera) {
    RCLCPP_ERROR_STREAM(get_logger(), "giving up, camera " << serial_ << " not found!");
    return (false);
  }
  keepRunning_ = true;
  thread_ = std::make_shared<std::thread>(&CameraDriver::run, this);

  if (wrapper_->initCamera(serial_)) {
    if (dumpNodeMap_) {
      RCLCPP_INFO_STREAM(get_logger(), "dumping node map!");
      std::string nm = wrapper_->getNodeMapAsString();
      std::cout << nm;
    }
    // Must first create the camera parameters before acquisition is started.
    // Some parameters (like blackfly s chunk control) cannot be set once
    // the camera is running.
    createCameraParameters();
    // TODO(bernd): once ROS2 supports subscriber status callbacks, this can go!
    startCamera();
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "init camera failed for cam: " << serial_);
  }
  return (true);
}
}  // namespace spinnaker_camera_driver

RCLCPP_COMPONENTS_REGISTER_NODE(spinnaker_camera_driver::CameraDriver)
