#!/usr/bin/env python3
"""
Launch robot_state_publisher + ros2_control + ForwardCommandController + RL policy + RViz.

Pipeline:
  rl_policy_node → /right_forward_position_controller/commands
     ↓
  forward_position_controller → hardware driver
     ↓
  joint_state_broadcaster → /joint_states
     ↓
  robot_state_publisher → /tf → RViz

Usage:
  ros2 launch openarm_sim2real sim2real_forward_cmd.launch.py \
    model_path:=/path/to/policy.pt

Use real hardware: use_fake_hardware:=false
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_robot_description(context, desc_pkg, desc_file, arm_type,
                               use_fake_hardware, right_can, left_can):
    desc_pkg_str = context.perform_substitution(desc_pkg)
    desc_file_str = context.perform_substitution(desc_file)
    arm_str = context.perform_substitution(arm_type)
    fake_str = context.perform_substitution(use_fake_hardware)
    right_can_str = context.perform_substitution(right_can)
    left_can_str = context.perform_substitution(left_can)

    xacro_path = os.path.join(
        get_package_share_directory(desc_pkg_str),
        "urdf", "robot", desc_file_str,
    )
    return xacro.process_file(xacro_path, mappings={
        "arm_type": arm_str,
        "bimanual": "true",
        "use_fake_hardware": fake_str,
        "ros2_control": "true",
        "right_can_interface": right_can_str,
        "left_can_interface": left_can_str,
    }).toprettyxml(indent="  ")


def generate_launch_description():
    declared = [
        DeclareLaunchArgument("description_package",
                              default_value="openarm_description"),
        DeclareLaunchArgument("description_file",
                              default_value="v10.urdf.xacro"),
        DeclareLaunchArgument("arm_type", default_value="v10"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("right_can_interface", default_value="can0"),
        DeclareLaunchArgument("left_can_interface", default_value="can1"),
        DeclareLaunchArgument("runtime_config_package",
                              default_value="openarm_bringup"),
        DeclareLaunchArgument("controllers_file",
                              default_value="openarm_v10_bimanual_controllers.yaml"),
        DeclareLaunchArgument("model_path",
                              default_value="",
                              description="Path to the RL policy model (.pt file)"),
    ]

    desc_pkg = LaunchConfiguration("description_package")
    desc_file = LaunchConfiguration("description_file")
    arm_type = LaunchConfiguration("arm_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    right_can = LaunchConfiguration("right_can_interface")
    left_can = LaunchConfiguration("left_can_interface")
    rt_pkg = LaunchConfiguration("runtime_config_package")
    ctrl_file = LaunchConfiguration("controllers_file")
    model_path = LaunchConfiguration("model_path")

    ctrl_path = PathJoinSubstitution(
        [FindPackageShare(rt_pkg), "config", "v10_controllers", ctrl_file])

    def robot_spawner(context: LaunchContext):
        rd = generate_robot_description(
            context, desc_pkg, desc_file, arm_type,
            use_fake_hardware, right_can, left_can)
        params = {"robot_description": rd}
        ctrl_str = context.perform_substitution(ctrl_path)
        return [
            Node(package="robot_state_publisher",
                 executable="robot_state_publisher",
                 output="screen",
                 parameters=[params]),
            Node(package="controller_manager",
                 executable="ros2_control_node",
                 output="both",
                 parameters=[params, ctrl_str]),
        ]

    jsb = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    fwd_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["right_forward_position_controller",
                   "--controller-manager", "/controller_manager"],
    )

    rviz = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare(desc_pkg), "rviz", "bimanual.rviz"])],
        output="log",
    )

    rl_policy = Node(
        package="openarm_sim2real",
        executable="rl_policy_node.py",
        name="rl_policy_node",
        output="screen",
        parameters=[{"model_path": model_path}],
    )

    delay = 3.0
    policy_delay = 5.0
    return LaunchDescription(declared + [
        OpaqueFunction(function=robot_spawner),
        rviz,
        TimerAction(period=delay, actions=[jsb]),
        TimerAction(period=delay, actions=[fwd_ctrl]),
        TimerAction(period=policy_delay, actions=[rl_policy]),
    ])
