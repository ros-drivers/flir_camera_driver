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

#include <image_transport/image_transport.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <spinnaker_camera_driver/camera_driver.hpp>
#include <spinnaker_camera_driver/logging.hpp>
#include <spinnaker_camera_driver/utils.hpp>

namespace spinnaker_camera_driver
{
CameraDriver::CameraDriver(const rclcpp::NodeOptions & options) : NodeType("camera_driver", options)
{
  const std::string cameraName = get_node_base_interface()->get_name();
#ifdef IMAGE_TRANSPORT_SUPPORTS_NODE_INTERFACES
  imageTransport_ = std::make_shared<ImageTransport>(image_transport::RequiredInterfaces(*this));
  infoManager_ = utils::makeCameraInfoManager(
    get_node_base_interface(), get_node_parameters_interface(), get_node_logging_interface(),
    get_node_services_interface(), cameraName, "camerainfo_url", 10);
#else
  imageTransport_ =
    std::make_shared<ImageTransport>(std::shared_ptr<CameraDriver>(this, [](auto *) {}));
  infoManager_ = utils::makeCameraInfoManager(this, cameraName, "camerainfo_url");
#endif
  camera_ = std::make_shared<Camera>(
    get_node_base_interface(), get_node_parameters_interface(), get_node_logging_interface(),
    get_node_timers_interface(), get_node_clock_interface(), get_node_topics_interface(),
    get_node_services_interface(), imageTransport_.get(), infoManager_.get(), cameraName, "");
#ifdef IMAGE_TRANSPORT_SUPPORTS_LIFECYCLE_NODE
  get_node_base_interface()->get_context()->add_pre_shutdown_callback(
    std::bind(&CameraDriver::preShutdown, this));

  if (declare_parameter<bool>("auto_start", true)) {
    // defer because one cannot call some of the required methods inside the constructor
    timer_ = create_wall_timer(std::chrono::microseconds(0), [this]() -> void {
      timer_->cancel();
      rclcpp_lifecycle::LifecycleNode::configure();
      rclcpp_lifecycle::LifecycleNode::activate();
    });
  }
#else
  camera_->configure();
  camera_->activate();
#endif
}

CameraDriver::~CameraDriver()
{
  if (timer_) {
    timer_->cancel();
    timer_->reset();
  }
  shutdown();  // no harm, should be idempotent
}

void CameraDriver::shutdown()
{
  if (camera_) {
    camera_->deactivate();
    camera_->deconfigure();
    try {
      camera_.reset();
      imageTransport_.reset();
      infoManager_.reset();
    } catch (const std::exception & e) {
      LOG_ERROR("error during shutdown: " << e.what());
    }
    LOG_INFO("shutdown complete.");
  }
}

#ifdef IMAGE_TRANSPORT_SUPPORTS_LIFECYCLE_NODE

void CameraDriver::preShutdown() { rclcpp_lifecycle::LifecycleNode::shutdown(); }

CbReturn CameraDriver::on_configure(const LCState & s)
{
  return (changeState(
    [s](CameraDriver & ob) -> CbReturn { return (ob.NodeType::on_configure(s)); },
    &Camera::configure, "configure"));
}

CbReturn CameraDriver::on_activate(const LCState & s)
{
  return (changeState(
    [s](CameraDriver & ob) -> CbReturn { return (ob.NodeType::on_activate(s)); }, &Camera::activate,
    "activate"));
}

CbReturn CameraDriver::on_deactivate(const LCState & s)
{
  return (changeState(
    [s](CameraDriver & ob) -> CbReturn { return (ob.NodeType::on_deactivate(s)); },
    &Camera::deactivate, "deactivate"));
}

CbReturn CameraDriver::on_cleanup(const LCState & s)
{
  auto ret = changeState(
    [s](CameraDriver & ob) -> CbReturn { return (ob.NodeType::on_cleanup(s)); },
    &Camera::deconfigure, "cleanup");
  return (ret);
}

CbReturn CameraDriver::on_shutdown(const LCState & state)
{
  auto ret = NodeType::on_shutdown(state);
  if (ret != CbReturn::SUCCESS) {
    return (ret);
  }
  shutdown();
  return CbReturn::SUCCESS;
}

CbReturn CameraDriver::on_error(const LCState & state)
{
  LOG_ERROR("got error state: " << state.label());
  return (CbReturn::FAILURE);
}

#endif

}  // namespace spinnaker_camera_driver

RCLCPP_COMPONENTS_REGISTER_NODE(spinnaker_camera_driver::CameraDriver)
