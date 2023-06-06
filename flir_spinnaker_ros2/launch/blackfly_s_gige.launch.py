# -----------------------------------------------------------------------------
# Copyright 2022 Kevin Janesch
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
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

camera_params = {
    'debug': False,
    'compute_brightness': False,
    'dump_node_map': False,
    # set parameters defined in blackfly_s_gige.cfg
    'gain_auto': 'Continuous',
    'exposure_auto': 'Continuous',
    'frame_rate_auto': 'Off',
    'frame_rate': 20.0,
    'frame_rate_enable': True,
    'trigger_mode': 'Off',
    'chunk_mode_active': True,
    'chunk_selector_frame_id': 'FrameID',
    'chunk_enable_frame_id': True,
    'chunk_selector_exposure_time': 'ExposureTime',
    'chunk_enable_exposure_time': True,
    'chunk_selector_gain': 'Gain',
    'chunk_enable_gain': True,
    'chunk_selector_timestamp': 'Timestamp',
    'chunk_enable_timestamp': True,
    'gev_scps_packet_size': 9000,
    }


def generate_launch_description():
    """Launch blackfly_s camera node."""
    flir_dir = get_package_share_directory('flir_spinnaker_ros2')
    config_dir = flir_dir + '/config/'
    name_arg = LaunchArg('camera_name', default_value='blackfly_s',
                         description='camera name')
    serial_arg = LaunchArg('serial', default_value="'20435008'",
                           description='serial number')
    print([LaunchConfig('serial'), '_'])
    node = Node(package='flir_spinnaker_ros2',
                executable='camera_driver_node',
                output='screen',
                name=[LaunchConfig('camera_name')],
                parameters=[
                    camera_params,
                    {'parameter_file': config_dir + 'blackfly_s_gige.cfg',
                     'serial_number': [LaunchConfig('serial')], }],
                remappings=[('~/control', '/exposure_control/control')])
    return LaunchDescription([name_arg, serial_arg, node])
