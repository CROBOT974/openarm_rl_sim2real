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

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    openarm_bimanual = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("openarm_bringup"),
                "launch",
                "openarm.bimanual.launch.py",
            )
        ),
        launch_arguments={
            "arm_type": "v10",
            "use_fake_hardware": "false",
        }.items(),
    )

    aruco_node = Node(
        package="openarm_calibration",
        executable="aruco_pose_node.py",
        name="aruco_pose_node",
        arguments=[
            "--camera-frame", "tr_base",
            "--marker-frame", "tr_marker",
            "--marker-id", "10",
            "--marker-size", "0.05",
        ],
    )

    handeye_server = Node(
        package="easy_handeye2",
        executable="handeye_server",
        name="handeye_server",
        parameters=[{
            "name": "openarm_right_eob",
            "calibration_type": "eye_on_base",
            "tracking_base_frame": "tr_base",
            "tracking_marker_frame": "tr_marker",
            "robot_base_frame": "world",
            "robot_effector_frame": "openarm_right_hand",
        }],
    )

    rqt_calibrator = Node(
        package="easy_handeye2",
        executable="rqt_calibrator.py",
        name="handeye_rqt_calibrator",
        parameters=[{
            "name": "openarm_right_eob",
            "calibration_type": "eye_on_base",
            "tracking_base_frame": "tr_base",
            "tracking_marker_frame": "tr_marker",
            "robot_base_frame": "world",
            "robot_effector_frame": "openarm_right_hand",
        }],
    )

    return LaunchDescription([
        openarm_bimanual,
        TimerAction(period=3.0, actions=[aruco_node]),
        TimerAction(period=5.0, actions=[handeye_server]),
        TimerAction(period=5.0, actions=[rqt_calibrator]),
    ])
