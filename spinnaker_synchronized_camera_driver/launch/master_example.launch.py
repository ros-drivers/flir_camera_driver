# -----------------------------------------------------------------------------
# Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
# Example file for two Blackfly S cameras that are *externally triggered*, i.e
# you must provide an external hardware synchronization pulse to both cameras!
#
# Each of them regulates their exposure time individually by instantiating a
# master exposure controller. This setup is useful when cameras are not facing
# the scene, or facing it from very different angles such that different
# lighting conditions must be expected.
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution as PJoin
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

camera_list = {
    'cam0': '20435008',
    'cam1': '20415937',
}

exposure_controller_parameters = {
    'type': 'master',
    'brightness_target': 120,  # from 0..255
    'brightness_tolerance': 20,  # when to update exposure/gain
    # watch that max_exposure_time is short enough
    # to support the trigger frame rate!
    'max_exposure_time': 15000,  # usec
    'min_exposure_time': 5000,  # usec
    'max_gain': 29.9,
    'gain_priority': False,
}

cam_parameters = {
    'debug': False,
    'quiet': True,
    'buffer_queue_size': 1,
    'compute_brightness': True,
    'exposure_auto': 'Off',
    'exposure_time': 10000,  # not used under auto exposure
    'trigger_mode': 'On',
    'gain_auto': 'Off',
    'trigger_source': 'Line3',
    'trigger_selector': 'FrameStart',
    'trigger_overlap': 'ReadOut',
    'trigger_activation': 'RisingEdge',
    'balance_white_auto': 'Continuous',
    # You must enable chunk mode and chunks: frame_id, exposure_time, and gain
    'chunk_mode_active': True,
    'chunk_selector_frame_id': 'FrameID',
    'chunk_enable_frame_id': True,
    'chunk_selector_exposure_time': 'ExposureTime',
    'chunk_enable_exposure_time': True,
    'chunk_selector_gain': 'Gain',
    'chunk_enable_gain': True,
    # The Timestamp is not used at the moment
    'chunk_selector_timestamp': 'Timestamp',
    'chunk_enable_timestamp': True,
}


def make_parameters(context):
    """Launch synchronized camera driver node."""
    pd = LaunchConfig('camera_parameter_directory')
    calib_url = 'file://' + LaunchConfig('calibration_directory').perform(context) + '/'

    exp_ctrl_names = [cam + '.exposure_controller' for cam in camera_list.keys()]
    driver_parameters = {
        'cameras': list(camera_list.keys()),
        'exposure_controllers': exp_ctrl_names,
        'ffmpeg_image_transport.encoding': 'hevc_nvenc',  # only for ffmpeg image transport
    }
    # generate identical exposure controller parameters for all cameras
    for exp in exp_ctrl_names:
        driver_parameters.update(
            {exp + '.' + k: v for k, v in exposure_controller_parameters.items()}
        )

    # generate camera parameters
    cam_parameters['parameter_file'] = PJoin([pd, 'blackfly_s.yaml'])
    for cam, serial in camera_list.items():
        cam_params = {cam + '.' + k: v for k, v in cam_parameters.items()}
        cam_params[cam + '.serial_number'] = serial
        cam_params[cam + '.camerainfo_url'] = calib_url + serial + '.yaml'
        cam_params[cam + '.frame_id'] = cam
        driver_parameters.update(cam_params)  # insert into main parameter list
        # link the camera to its exposure controller. Each camera has its own controller
        driver_parameters.update({cam + '.exposure_controller_name': cam + '.exposure_controller'})
    return driver_parameters


def launch_setup(context, *args, **kwargs):
    container = ComposableNodeContainer(
        name='cam_sync_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='spinnaker_synchronized_camera_driver',
                plugin='spinnaker_synchronized_camera_driver::SynchronizedCameraDriver',
                name='cam_sync',
                parameters=[make_parameters(context)],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )  # end of container
    return [container]


def generate_launch_description():
    return LaunchDescription(
        [
            LaunchArg(
                'camera_parameter_directory',
                default_value=PJoin([FindPackageShare('spinnaker_camera_driver'), 'config']),
                description='root directory for camera parameter definitions',
            ),
            LaunchArg(
                'calibration_directory',
                default_value=['camera_calibrations'],
                description='root directory for camera calibration files',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
