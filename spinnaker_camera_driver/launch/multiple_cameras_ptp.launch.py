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

#
# Example launch script for using PTP with two Blackfly S GigE cameras connected
# via shared 1Gb connection to the ROS host.
#
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

camera_params = {
    'debug': False,
    # 'device_link_throughput_limit': 125000000,  # 1Gbit
    # The two cameras are connected through a single gigabit link,
    # so cut the bandwidth by half to avoid incomplete messages
    'device_link_throughput_limit': 62500000,   # in MB/sec
    'gev_scps_packet_size': 1400,   # intel nics have bug that prevents 9000
    'gev_ieee_1588': True,  # enable PTP on the camera cameras
    'gev_ieee_1588_mode': 'SlaveOnly',  # 'SlaveOnly',  'Auto',
    'use_ieee_1588': True,   # actually use the PTP time as header stamp
    'compute_brightness': True,
    'dump_node_map': False,
    'adjust_timestamp': False,
    'gain_auto': 'Off',
    'gain': 0,
    'exposure_auto': 'Off',
    'exposure_time': 9000,
    'line2_selector': 'Line2',
    'line2_v33enable': False,
    'line3_selector': 'Line3',
    'line3_linemode': 'Input',
    'trigger_selector': 'FrameStart',
    'trigger_mode': 'Off',  # Switch this to "On" to use external hardware trigger
    'trigger_source': 'Line3',
    'trigger_overlap': 'ReadOut',
    'chunk_mode_active': True,
    'chunk_selector_frame_id': 'FrameID',
    'chunk_enable_frame_id': True,
    'chunk_selector_exposure_time': 'ExposureTime',
    'chunk_enable_exposure_time': True,
    'chunk_selector_gain': 'Gain',
    'chunk_enable_gain': True,
    'chunk_selector_timestamp': 'Timestamp',
    'chunk_enable_timestamp': True,
}


def make_camera_node(name, camera_type, serial):
    parameter_file = PathJoinSubstitution(
        [FindPackageShare('spinnaker_camera_driver'), 'config', camera_type + '.yaml']
    )

    node = ComposableNode(
        package='spinnaker_camera_driver',
        plugin='spinnaker_camera_driver::CameraDriver',
        name=name,
        parameters=[camera_params, {'parameter_file': parameter_file, 'serial_number': serial}],
        remappings=[
            ('~/control', '/exposure_control/control'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    return node


def launch_setup(context, *args, **kwargs):
    """Create multiple camera."""
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
            make_camera_node(
                LaunchConfig('cam_0_name'),
                LaunchConfig('cam_0_type').perform(context),
                LaunchConfig('cam_0_serial'),
            ),
            make_camera_node(
                LaunchConfig('cam_1_name'),
                LaunchConfig('cam_1_type').perform(context),
                LaunchConfig('cam_1_serial'),
            ),
        ],
        output='screen',
    )  # end of container
    return [container]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return LaunchDescription(
        [
            LaunchArg(
                'cam_0_name',
                default_value=['cam_0'],
                description='camera name (ros node name) of camera 0',
            ),
            LaunchArg(
                'cam_1_name',
                default_value=['cam_1'],
                description='camera name (ros node name) of camera 1',
            ),
            LaunchArg('cam_0_type', default_value='blackfly_s', description='type of camera 0'),
            LaunchArg('cam_1_type', default_value='blackfly_s', description='type of camera 1'),
            LaunchArg(
                'cam_0_serial',
                default_value="'20255407'",
                description='FLIR serial number of camera 0 (in quotes!!)',
            ),
            LaunchArg(
                'cam_1_serial',
                default_value="'20255396'",
                description='FLIR serial number of camera 1 (in quotes!!)',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
