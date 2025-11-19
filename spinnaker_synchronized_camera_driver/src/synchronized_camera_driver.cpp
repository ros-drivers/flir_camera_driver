// -*-c++-*--------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include <image_transport/image_transport.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <spinnaker_camera_driver/exposure_controller.hpp>
#include <spinnaker_camera_driver/lifecycle_types.hpp>
#include <spinnaker_camera_driver/utils.hpp>
#include <spinnaker_synchronized_camera_driver/exposure_controller_factory.hpp>
#include <spinnaker_synchronized_camera_driver/logging.hpp>
#include <spinnaker_synchronized_camera_driver/synchronized_camera_driver.hpp>
#include <spinnaker_synchronized_camera_driver/time_estimator.hpp>

namespace spinnaker_synchronized_camera_driver
{
SynchronizedCameraDriver::SynchronizedCameraDriver(const rclcpp::NodeOptions & options)
: NodeType("sync_cam_driver", options), timeEstimator_(new TimeEstimator())
{
#ifdef IMAGE_TRANSPORT_SUPPORTS_LIFECYCLE_NODE
  imageTransport_ = std::make_shared<ImageTransport>(image_transport::RequiredInterfaces(*this));
  get_node_base_interface()->get_context()->add_pre_shutdown_callback(
    std::bind(&SynchronizedCameraDriver::preShutdown, this));
  if (declare_parameter("auto_start", true)) {
    // defer because one cannot call some of the required methods inside the constructor
    timer_ = create_wall_timer(std::chrono::microseconds(0), [this]() -> void {
      timer_->cancel();
      rclcpp_lifecycle::LifecycleNode::configure();
      rclcpp_lifecycle::LifecycleNode::activate();
    });
  }
#else
  imageTransport_ = std::make_shared<ImageTransport>(
    std::shared_ptr<SynchronizedCameraDriver>(this, [](auto *) {}));
  if (configure()) {
    activate();
  }
#endif
}

SynchronizedCameraDriver::~SynchronizedCameraDriver() { shutdown(); }

bool SynchronizedCameraDriver::configure()
{
  if (!createExposureControllers()) {
    return (false);
  }
  if (!createCameras()) {
    return (false);
  }
  for (auto & c : cameras_) {
    if (!c.second->configure()) {
      return (false);
    }
  }
  return (true);
}

bool SynchronizedCameraDriver::activate()
{
  for (auto & c : cameras_) {
    if (!c.second->activate()) {
      return (false);
    }
  }
  statusTimer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Duration(5, 0),
    std::bind(&SynchronizedCameraDriver::printStatus, this));
  return (true);
}

bool SynchronizedCameraDriver::deactivate()
{
  for (auto & c : cameras_) {
    if (!c.second->deactivate()) {
      return (false);
    }
  }
  if (statusTimer_ && !statusTimer_->is_canceled()) {
    statusTimer_->cancel();
  }
  return (true);
}

bool SynchronizedCameraDriver::deconfigure()
{
  for (auto & c : cameras_) {
    if (!c.second->deconfigure()) {
      return (false);
    }
  }
  destroyCameras();
  destroyExposureControllers();
  return (true);
}

void SynchronizedCameraDriver::shutdown()
{
  if (timer_) {
    timer_->cancel();
    timer_->reset();
  }
  (void)deactivate();
  (void)deconfigure();
}

void SynchronizedCameraDriver::printStatus()
{
  if (numUpdatesReceived_ < numUpdatesRequired_) {
    LOG_INFO("waiting for accurate frequency, current estimate: " << 1.0 / avgFrameInterval_);
    return;
  }

  struct TKInfo
  {
    explicit TKInfo(const std::string & n, double off, double jit, int64_t d, size_t i)
    : name(n), offset(off), jitter(jit), dropped(d), incomplete(i)
    {
    }
    std::string name;
    double offset;
    double jitter;
    int64_t dropped;
    size_t incomplete;
  };
  std::vector<TKInfo> tki;
  double dt = 0;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    dt = avgFrameInterval_;
    for (auto & tk : timeKeepers_) {
      tki.push_back(TKInfo(
        tk->getName(), tk->getOffsetAverage() / dt, std::sqrt(tk->getOffsetVariance()) / dt,
        tk->getNumFramesDropped(), tk->getNumFramesIncomplete()));
      tk->clearStatistics();
    }
  }
  LOG_INFO_FMT("------ frequency: %10.3f Hz", 1.0 / dt);
  LOG_INFO_FMT("%-8s %4s %4s %9s %9s", "camera", "drop", "icmp", "offset", "jitter");
  for (auto & tk : tki) {
    LOG_INFO_FMT(
      "%-8s %4ld %4zu %8.2f%% %8.2f%%", tk.name.c_str(), tk.dropped, tk.incomplete, tk.offset * 100,
      tk.jitter * 100);
  }
}

bool SynchronizedCameraDriver::createExposureControllers()
{
  using svec = std::vector<std::string>;
  const svec controllers = this->declare_parameter<svec>("exposure_controllers", svec());
  for (const auto & c : controllers) {
    const std::string type = this->declare_parameter<std::string>(c + ".type", "");
    if (!type.empty()) {
      exposureControllers_.insert(
        {c,
         exposure_controller_factory::newInstance(type, c, this->get_node_parameters_interface())});
      LOG_INFO("created exposure controller: " << c);
    } else {
      BOMB_OUT("no controller type specified for controller " << c);
    }
  }
  // allow the exposure controllers to link to each other.
  for (auto & c : exposureControllers_) {
    c.second->link(exposureControllers_);
  }
  return (true);
}

bool SynchronizedCameraDriver::createCameras()
{
  using svec = std::vector<std::string>;
  const svec cameras = this->declare_parameter<svec>("cameras", svec());
  if (cameras.empty()) {
    BOMB_OUT("no cameras configured for synchronized driver!");
  }
  for (size_t i = 0; i < cameras.size(); i++) {
    const auto & c = cameras[i];
#ifdef IMAGE_TRANSPORT_SUPPORTS_NODE_INTERFACES
    auto mgr = spinnaker_camera_driver::utils::makeCameraInfoManager(
      get_node_base_interface(), get_node_parameters_interface(), get_node_logging_interface(),
      get_node_services_interface(), c, c + ".camerainfo_url", 10);
#else
    auto mgr =
      spinnaker_camera_driver::utils::makeCameraInfoManager(this, c, c + ".camerainfo_url");
#endif
    infoManagers_.push_back(mgr);
    auto cam = std::make_shared<spinnaker_camera_driver::Camera>(
      get_node_base_interface(), get_node_parameters_interface(), get_node_logging_interface(),
      get_node_timers_interface(), get_node_clock_interface(), get_node_topics_interface(),
      get_node_services_interface(), imageTransport_.get(), mgr.get(), c, c, false);
    cameras_.insert({c, cam});
    timeKeepers_.push_back(std::make_shared<TimeKeeper>(i, c, this));
    cam->setSynchronizer(timeKeepers_.back());
    // set exposure controller if configured
    const auto ctrlName = this->declare_parameter<std::string>(c + ".exposure_controller_name", "");
    if (!ctrlName.empty()) {
      auto it = exposureControllers_.find(ctrlName);
      if (it == exposureControllers_.end()) {
        BOMB_OUT("unknown exposure controller: " << ctrlName);
      }
      it->second->addCamera(cam);
      cam->setExposureController(it->second);
    }
  }
  numUpdatesRequired_ = cameras.size() * 3;
  return (true);
}

void SynchronizedCameraDriver::destroyExposureControllers() { exposureControllers_.clear(); }

void SynchronizedCameraDriver::destroyCameras()
{
  cameras_.clear();
  timeKeepers_.clear();
  infoManagers_.clear();
}

bool SynchronizedCameraDriver::update(
  size_t idx, uint64_t hostTime, double dt, uint64_t * frameTime)
{
  std::unique_lock<std::mutex> lock(mutex_);
  constexpr double NUM_FRAMES_TO_AVG = 20.0;
  constexpr double alpha = 1.0 / NUM_FRAMES_TO_AVG;
  dt = std::max(1e-6, dt);
  avgFrameInterval_ =
    (avgFrameInterval_ < 0) ? dt : (avgFrameInterval_ * (1.0 - alpha) + alpha * dt);
  if (numUpdatesReceived_ < numUpdatesRequired_) {
    numUpdatesReceived_++;
    if (numUpdatesReceived_ >= numUpdatesRequired_) {
      timeEstimator_->initialize(hostTime, avgFrameInterval_);
    }
    *frameTime = hostTime;
    return (true);
  }
  const bool gotTime = timeEstimator_->update(idx, hostTime, frameTime);
  return (gotTime);
}

#ifdef IMAGE_TRANSPORT_SUPPORTS_LIFECYCLE_NODE

void SynchronizedCameraDriver::preShutdown()
{
  LOG_INFO("running preShutdown()");
  rclcpp_lifecycle::LifecycleNode::shutdown();
}

CbReturn SynchronizedCameraDriver::on_configure(const LCState & s)
{
  LOG_INFO("requested configure()");
  const auto ret = NodeType::on_configure(s);
  if (ret != CbReturn::SUCCESS) {
    return (ret);
  }
  return (configure() ? CbReturn::SUCCESS : CbReturn::FAILURE);
}

CbReturn SynchronizedCameraDriver::on_activate(const LCState & s)
{
  LOG_INFO("requested activate()");
  const auto ret = NodeType::on_activate(s);
  if (ret != CbReturn::SUCCESS) {
    return (ret);
  }
  return (activate() ? CbReturn::SUCCESS : CbReturn::FAILURE);
}

CbReturn SynchronizedCameraDriver::on_deactivate(const LCState & s)
{
  LOG_INFO("requested deactivate()");
  const auto ret = NodeType::on_deactivate(s);
  if (ret != CbReturn::SUCCESS) {
    return (ret);
  }
  return (deactivate() ? CbReturn::SUCCESS : CbReturn::FAILURE);
}

CbReturn SynchronizedCameraDriver::on_cleanup(const LCState & s)
{
  LOG_INFO("requested cleanup()");
  const auto ret = NodeType::on_cleanup(s);
  if (ret != CbReturn::SUCCESS) {
    return (ret);
  }
  return (deconfigure() ? CbReturn::SUCCESS : CbReturn::FAILURE);
}

CbReturn SynchronizedCameraDriver::on_shutdown(const LCState & state)
{
  LOG_INFO("requested shutdown()");
  auto ret = NodeType::on_shutdown(state);
  if (ret != CbReturn::SUCCESS) {
    return (ret);
  }
  shutdown();
  return CbReturn::SUCCESS;
}

CbReturn SynchronizedCameraDriver::on_error(const LCState & state)
{
  LOG_ERROR("got error state: " << state.label());
  return (CbReturn::FAILURE);
}
#endif

}  // namespace spinnaker_synchronized_camera_driver

RCLCPP_COMPONENTS_REGISTER_NODE(spinnaker_synchronized_camera_driver::SynchronizedCameraDriver)
