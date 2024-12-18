# -----------------------------------------------------------------------------
# Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com> and Li Dahua
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

# Example file for two Blackfly S cameras where the primary camera triggers
# the secondary via its 3.3V signaling interface.
#
# One of them creates a master controller, the other one a follower. The exposure
# parameters are determined by the master. This is a useful setup for e.g. a
# synchronized stereo camera.
#
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution as PJoin
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

# Camera list with serial numbers
camera_list = {
    'cam0': '16378750',
    'cam1': '16378734',
}

# Exposure controller parameters
exposure_controller_parameters = {
    'brightness_target': 50,  # from 0..255
    'brightness_tolerance': 20,  # when to update exposure/gain
    'max_exposure_time': 15000,  # usec
    'min_exposure_time': 2000,  # usec
    'max_gain': 29.9,
    'gain_priority': False,
}

# Parameters shared by all cameras
shared_cam_parameters = {
    'debug': False,
    'quiet': True,
    'buffer_queue_size': 1,
    'compute_brightness': True,
    'pixel_format': 'BGR8',
    'exposure_auto': 'Off',
    'exposure_time': 2083.333,  # not used under auto exposure
    'gain_auto': 'Off',
    'balance_white_auto': 'Continuous',
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

# Parameters for the primary camera
primary_cam_parameters = {
    **shared_cam_parameters,
    'trigger_mode': 'On',
    'trigger_source': 'Line0',
    'trigger_selector': 'FrameStart',
    'trigger_overlap': 'ReadOut',
    'line0_selector': 'Line0',
    'line0_linemode': 'Input',
    # 'line2_selector': 'Line2',
    # 'line2_v33enable': True,
}

# Parameters for the secondary camera
secondary_cam_parameters = {
    **shared_cam_parameters,
    'trigger_mode': 'On',
    'trigger_source': 'Line0',
    'trigger_selector': 'FrameStart',
    'trigger_overlap': 'ReadOut',
}


def make_parameters(context):
    """Generate camera parameters for the driver node."""
    pd = LaunchConfig('camera_parameter_directory')
    calib_url = 'file://' + LaunchConfig('calibration_directory').perform(context) + '/'

    exp_ctrl_names = [cam + '.exposure_controller' for cam in camera_list.keys()]
    driver_parameters = {
        'cameras': list(camera_list.keys()),
        'exposure_controllers': exp_ctrl_names,
        'ffmpeg_image_transport.encoding': 'hevc_nvenc',
    }
    # Generate identical exposure controller parameters for all cameras
    for exp in exp_ctrl_names:
        driver_parameters.update(
            {exp + '.' + k: v for k, v in exposure_controller_parameters.items()}
        )

    # Set cam0 as master and cam1 as follower
    driver_parameters[exp_ctrl_names[0] + '.type'] = 'master'
    driver_parameters[exp_ctrl_names[1] + '.type'] = 'follower'
    driver_parameters[exp_ctrl_names[1] + '.master'] = exp_ctrl_names[0]

    # Generate camera parameters
    primary_cam_parameters['parameter_file'] = PJoin([pd, '16378750.yaml'])
    secondary_cam_parameters['parameter_file'] = PJoin([pd, '16378734.yaml'])
    for cam, serial in camera_list.items():
        if cam == 'cam0':
            cam_params = {cam + '.' + k: v for k, v in primary_cam_parameters.items()}
        else:
            cam_params = {cam + '.' + k: v for k, v in secondary_cam_parameters.items()}
        cam_params[cam + '.serial_number'] = serial
        cam_params[cam + '.camerainfo_url'] = calib_url + serial + '.yaml'
        cam_params[cam + '.frame_id'] = cam
        driver_parameters.update(cam_params)
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
    )
    return [container]


def generate_launch_description():
    return LaunchDescription(
        [
            LaunchArg(
                'camera_parameter_directory',
                default_value=PJoin([FindPackageShare('spinnaker_camera_driver'), 'config']),
                description='Root directory for camera parameter definitions',
            ),
            LaunchArg(
                'calibration_directory',
                default_value=['camera_calibrations'],
                description='Root directory for camera calibration files',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
