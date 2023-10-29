# Copyright 2021 Clearpath Robotics, Inc.
# Copyright 2023 Bernd Pfrommer
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
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)
# @author Bernd Pfrommer (bernd@pfrommer.us)


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='flir_blackfly_s',
                          choices=['flir_blackfly_s'],
                          description='camera model'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('camera_name', default_value=LaunchConfiguration('model'),
                          description='camera name'),
    DeclareLaunchArgument('namespace', default_value=LaunchConfiguration('camera_name'),
                          description='camera namespace'),
]


def generate_launch_description():
    camera_description = get_package_share_directory('flir_camera_description')
    xacro_file = PathJoinSubstitution([camera_description, 'urdf', 'demo.urdf.xacro'])
    namespace = LaunchConfiguration('namespace')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': Command([
                'xacro', ' ', xacro_file, ' ',
                'model:=', LaunchConfiguration('model'), ' ',
                'namespace:=', namespace])},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    world_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '1', '--yaw', '0',
                   '--pitch', '0', '--roll', '0',
                   '--frame-id', 'world', '--child-frame-id', 'camera_frame'])

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Add nodes to LaunchDescription
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(world_tf_publisher)

    return ld
