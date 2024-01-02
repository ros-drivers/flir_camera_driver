# -----------------------------------------------------------------------------
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution as PJoin
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

cam_parameters = {
    "parameter_file": "blackfly_s.yaml",
    "buffer_queue_size": 1,
    "frame_rate_auto": "Off",
    "frame_rate_enable": False,
    # watch that your exposure time is
    # short enough to support the trigger frame rate!
    "exposure_auto": "Off",
    "exposure_time": 6000,
    "gain_auto": "Continuous",
    "trigger_mode": "On",
    "trigger_source": "Line3",
    "trigger_selector": "FrameStart",
    "trigger_overlap": "ReadOut",
    "trigger_activation": "RisingEdge",
    # You must enable chunk mode and enable frame_id
    "chunk_mode_active": True,
    "chunk_selector_frame_id": "FrameID",
    "chunk_enable_frame_id": True,
    # The other chunk info should not be required
    # "chunk_selector_exposure_time": "ExposureTime",
    # "chunk_enable_exposure_time": True,
    # "chunk_selector_gain": "Gain",
    # "chunk_enable_gain": True,
    # "chunk_selector_timestamp": "Timestamp",
    # "chunk_enable_timestamp": True,
}


def launch_setup(context, *args, **kwargs):
    """Launch synchronized camera driver node."""
    pd = LaunchConfig("camera_parameter_directory")
    c0, c1 = "cam_0", "cam_1"
    c0p, c1p = c0 + ".", c1 + "."
    driver_parameters = {"cameras": [c0, c1]}
    cam_parameters["parameter_file"] = PJoin([pd, "blackfly_s.yaml"])
    cam_0_parameters = {c0p + k: v for k, v in cam_parameters.items()}
    cam_0_parameters[c0p + "serial_number"] = "20435008"
    cam_1_parameters = {c1p + k: v for k, v in cam_parameters.items()}
    cam_1_parameters[c1p + "serial_number"] = "20415937"
    node = Node(
        package="spinnaker_synchronized_camera_driver",
        executable="synchronized_camera_driver_node",
        output="screen",
        # prefix=["xterm -e gdb -ex run --args"],
        name=[LaunchConfig("driver_name")],
        parameters=[
            driver_parameters,
            cam_0_parameters,
            cam_1_parameters,
        ],
    )
    return [node]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return LaunchDescription(
        [
            LaunchArg(
                "driver_name",
                default_value=["cam_sync"],
                description="name of driver node",
            ),
            LaunchArg(
                "camera_parameter_directory",
                default_value=PJoin(
                    [FindPackageShare("spinnaker_camera_driver"), "config"]
                ),
                description="root directory for camera parameter definitions",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
