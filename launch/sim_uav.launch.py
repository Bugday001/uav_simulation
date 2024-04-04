# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    current_pkg = FindPackageShare('uav_simulation')
    use_sim_time = False
    package_name = 'uav_simulation'
    urdf_name = "uav_model.urdf"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'models/{urdf_name}')
    robot_description = ParameterValue(Command(['xacro ', urdf_model_path]), value_type=str)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': use_sim_time}]
        )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_model_path],
        parameters=[{'use_sim_time': use_sim_time}],
        )
    
    yaml_path = PathJoinSubstitution([current_pkg, 'config', 'cfg.yaml'])
    uav_sim_node = Node(
        name='uav_sim_node',
        package="uav_simulation",
        executable="uav_sim_node",
        parameters=[yaml_path],
        output='screen',
    )
    uav_ctrl_node = Node(
        package="uav_simulation",
        parameters=[{'use_sim_time': use_sim_time}],
        executable="uav_ctrl_node",
        output='screen',
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=["-d", "/home/bugday/Projects/test_ws/src/uav_simulation/launch/simulation.rviz"]
    )
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        uav_sim_node,
        # uav_ctrl_node,
        rviz2
    ])