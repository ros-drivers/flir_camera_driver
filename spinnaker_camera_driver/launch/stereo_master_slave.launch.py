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

from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch import LaunchDescription

# primary camera
camera_params1 = {
    #'pixel_format': "BGR8",
    #'frame_rate_enable': True,
    #'frame_rate': 20,
    'image_width': 2048,
    'image_height': 1536,
    'debug': False,
    'compute_brightness': True,
    'dump_node_map': False,
    'gain_auto': 'Off',
    'gain': 0,
    'exposure_auto': 'Off',
    'exposure_time': 9000,
    'line1_selector': 'Line1',
    'line1_linemode': 'Output',
    'line2_selector': 'Line2',
    'line2_v33enable': True,
    'trigger_mode': 'Off',
    'chunk_selector_frame_id': 'FrameID',
    'chunk_enable_frame_id': True,
    'chunk_selector_exposure_time': 'ExposureTime',
    'chunk_enable_exposure_time': True,
    'chunk_selector_gain': 'Gain',
    'chunk_enable_gain': True,
    'adjust_timestamp': True,
    'chunk_mode_active': True,
    'chunk_selector_timestamp': 'Timestamp',
    'chunk_enable_timestamp': True,
    }

# secondary camera
camera_params2 = {
    #'pixel_format': "BGR8",
    #'frame_rate_enable': True,
    #'frame_rate': 20,
    'image_width': 2048,
    'image_height': 1536,
    'debug': False,
    #'compute_brightness': True,
    'dump_node_map': False,
    'gain_auto': 'Off',
    'gain': 0,
    'exposure_auto': 'Off',
    'exposure_time': 9000,
    'trigger_selector': 'FrameStart',
    'trigger_mode': 'On',
    'trigger_source': 'Line3',
    'trigger_overlap': 'ReadOut',
    'chunk_selector_frame_id': 'FrameID',
    'chunk_enable_frame_id': True,
    'chunk_selector_exposure_time': 'ExposureTime',
    'chunk_enable_exposure_time': True,
    'chunk_selector_gain': 'Gain',
    'chunk_enable_gain': True,
    'adjust_timestamp': True,
    'chunk_mode_active': True,
    'chunk_selector_timestamp': 'Timestamp',
    'chunk_enable_timestamp': True,
    }

def make_camera_node(name, camera_type, serial, arch_type):
    parameter_file = PathJoinSubstitution(
        [FindPackageShare('spinnaker_camera_driver'), 'config',
         camera_type + '.yaml'])

    camera_params = camera_params1
    if arch_type == "master":
        camera_params = camera_params1
    else:
        camera_params = camera_params2

    node = ComposableNode(
        package='spinnaker_camera_driver',
        plugin='spinnaker_camera_driver::CameraDriver',
        name=name,
        parameters=[camera_params,
                    {'parameter_file': parameter_file,
                     'serial_number': serial}],
        remappings=[('~/control', '/exposure_control/control'), ],
        extra_arguments=[{'use_intra_process_comms': True}])
    return node


def launch_setup(context, *args, **kwargs):
    """Create synchronized stereo camera."""
    container = ComposableNodeContainer(
            name='stereo_camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                #
                # These two camera nodes run independently from each other,
                # but in the same address space
                #
                make_camera_node(LaunchConfig('cam_0_name'),
                                 LaunchConfig('cam_0_type').perform(context),
                                 LaunchConfig('cam_0_serial'), "master"),
                make_camera_node(LaunchConfig('cam_1_name'),
                                 LaunchConfig('cam_1_type').perform(context),
                                 LaunchConfig('cam_1_serial'), "slave"),
                #
                # This node is for external exposure control. Remove
                # if you don't need it, and switch on auto exposure.
                #
                ComposableNode(
                    package='exposure_control_ros2',
                    plugin='exposure_control_ros2::ExposureControl',
                    name='exposure_control',
                    parameters=[{'cam_name': LaunchConfig('cam_0_name'),
                                 'max_gain': 20.0,
                                 'gain_priority': False,
                                 'brightness_target': 100,
                                 'max_exposure_time': 9500.0,
                                 'min_exposure_time': 1000.0}],
                    remappings=[('~/meta', ['/', LaunchConfig('cam_0_name'),
                                            '/meta']), ],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
    )  # end of container
    return [container]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return LaunchDescription([
        LaunchArg('cam_0_name', default_value=['cam_0'],
                  description='camera name (ros node name) of camera 0'),
        LaunchArg('cam_1_name', default_value=['cam_1'],
                  description='camera name (ros node name) of camera 1'),
        LaunchArg('cam_0_type', default_value='blackfly_s',
                  description='type of camera 0'),
        LaunchArg('cam_1_type', default_value='blackfly_s',
                  description='type of camera 1'),
        LaunchArg('cam_0_serial', default_value="'21143313'",
                  description='FLIR serial number of camera 0 (in quotes!!)'),
        LaunchArg('cam_1_serial', default_value="'21143314'",
                  description='FLIR serial number of camera 1 (in quotes!!)'),
        OpaqueFunction(function=launch_setup)
        ])