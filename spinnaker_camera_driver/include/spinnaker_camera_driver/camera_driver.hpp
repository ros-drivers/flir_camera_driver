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

#ifndef SPINNAKER_CAMERA_DRIVER__CAMERA_DRIVER_HPP_
#define SPINNAKER_CAMERA_DRIVER__CAMERA_DRIVER_HPP_

#include <functional>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <spinnaker_camera_driver/camera.hpp>
#include <spinnaker_camera_driver/lifecycle_types.hpp>

namespace spinnaker_camera_driver
{
class CameraDriver : public NodeType
{
public:
  using CameraInfoManager = camera_info_manager::CameraInfoManager;
  using ImageTransport = image_transport::ImageTransport;
  explicit CameraDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraDriver() override;
#ifdef IMAGE_TRANSPORT_SUPPORTS_LIFECYCLE_NODE
protected:
  void preShutdown();
  CbReturn on_configure(const LCState & state) override;
  CbReturn on_activate(const LCState & state) override;
  CbReturn on_deactivate(const LCState & state) override;
  CbReturn on_cleanup(const LCState & state) override;
  CbReturn on_shutdown(const LCState & state) override;
  CbReturn on_error(const LCState & state) override;
  // helper methods
  using CameraFnPtr = bool (Camera::*)();
  template <typename F>
  CbReturn changeState(F && f, const CameraFnPtr & cameraFn, const std::string & trans)
  {
    RCLCPP_INFO_STREAM(get_logger(), "requested transition: " << trans);
    const auto ret = f(*this);
    if (ret != CbReturn::SUCCESS) {
      return (ret);
    }
    if (!camera_) {
      return (CbReturn::FAILURE);
    }
    const bool v = (camera_.get()->*cameraFn)();
    return (v ? CbReturn::SUCCESS : CbReturn::FAILURE);
  }
#endif

private:
  template <class T>
  T safeDeclare(const std::string & name, const T & def)
  {
    try {
      return (get_node_parameters_interface()
                ->declare_parameter(name, rclcpp::ParameterValue(def))
                .get<T>());
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
      const rclcpp::Parameter p = get_node_parameters_interface()->get_parameter(name);
      return (p.get_parameter_value().get<T>());
    }
  }

  void shutdown();
  // --------------- variables
  std::shared_ptr<ImageTransport> imageTransport_;
  std::shared_ptr<CameraInfoManager> infoManager_;
  std::shared_ptr<Camera> camera_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace spinnaker_camera_driver
#endif  // SPINNAKER_CAMERA_DRIVER__CAMERA_DRIVER_HPP_
