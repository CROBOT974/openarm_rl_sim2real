# Copyright 2025 Enactic, Inc.
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

"""
OpenArm underwater (upside-down mounting) launch file.
One arm, rotated 180° around X, mounted at world z=+1.2m via invisible body link.
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_robot_description(context, description_package, description_file,
                               arm_type, use_fake_hardware, can_interface, arm_prefix):
    description_package_str = context.perform_substitution(description_package)
    description_file_str = context.perform_substitution(description_file)
    arm_type_str = context.perform_substitution(arm_type)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    can_interface_str = context.perform_substitution(can_interface)
    arm_prefix_str = context.perform_substitution(arm_prefix)

    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "urdf", "robot", description_file_str
    )

    robot_description = xacro.process_file(xacro_path, mappings={
        "arm_type": arm_type_str,
        "underwater": "true",
        "use_fake_hardware": use_fake_hardware_str,
        "ros2_control": "true",
        "can_interface": can_interface_str,
        "arm_prefix": arm_prefix_str,
    }).toprettyxml(indent="  ")

    return robot_description


def robot_nodes_spawner(context, description_package, description_file,
                        arm_type, use_fake_hardware, controllers_file, can_interface, arm_prefix):
    robot_description = generate_robot_description(
        context, description_package, description_file,
        arm_type, use_fake_hardware, can_interface, arm_prefix
    )
    controllers_file_str = context.perform_substitution(controllers_file)
    param = {"robot_description": robot_description}

    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", output="screen", parameters=[param]),
        Node(package="controller_manager", executable="ros2_control_node",
             output="both", parameters=[param, controllers_file_str]),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("description_package", default_value="openarm_description"),
        DeclareLaunchArgument("description_file", default_value="v10.urdf.xacro"),
        DeclareLaunchArgument("arm_type", default_value="v10"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("robot_controller", default_value="joint_trajectory_controller",
                              choices=["forward_position_controller", "joint_trajectory_controller"]),
        DeclareLaunchArgument("runtime_config_package", default_value="openarm_bringup"),
        DeclareLaunchArgument("arm_prefix", default_value=""),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("controllers_file", default_value="openarm_v10_controllers.yaml"),
    ]

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    arm_type = LaunchConfiguration("arm_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_controller = LaunchConfiguration("robot_controller")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    can_interface = LaunchConfiguration("can_interface")
    arm_prefix = LaunchConfiguration("arm_prefix")

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", "v10_controllers", controllers_file])

    robot_nodes_spawner_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[description_package, description_file, arm_type,
              use_fake_hardware, controllers_file, can_interface, arm_prefix])

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "arm_underwater.rviz"])

    rviz_node = Node(package="rviz2", executable="rviz2", name="rviz2",
                     output="log", arguments=["-d", rviz_config_file])

    jsp = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])
    rc = Node(package="controller_manager", executable="spawner",
              arguments=[robot_controller, "-c", "/controller_manager"])
    gc = Node(package="controller_manager", executable="spawner",
              arguments=["gripper_controller", "-c", "/controller_manager"])

    D = 1.0
    return LaunchDescription(declared_arguments + [
        robot_nodes_spawner_func, rviz_node,
        TimerAction(period=D, actions=[jsp]),
        TimerAction(period=D, actions=[rc]),
        TimerAction(period=D, actions=[gc]),
    ])
