#!/usr/bin/env python3

import sys
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


# 7 joints of right arm
JOINTS = [
    "openarm_right_joint1",
    "openarm_right_joint2",
    "openarm_right_joint3",
    "openarm_right_joint4",
    "openarm_right_joint5",
    "openarm_right_joint6",
    "openarm_right_joint7",
]


class AutoSampler(Node):
    def __init__(self, poses):
        super().__init__("auto_sampler")
        self.poses = poses
        self.current = 0
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/right_joint_trajectory_controller/follow_joint_trajectory",
        )

    def next_pose(self):
        # All poses have been sampled
        if self.current >= len(self.poses):
            self.get_logger().info("All poses sampled!")
            rclpy.shutdown()
            return

        pose = self.poses[self.current]
        self.get_logger().info(
            f"--- Pose {self.current + 1}/{len(self.poses)}: "
            f"positions={pose['positions']}, time={pose.get('time', 3)}s ---"
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = pose["positions"]
        point.time_from_start.sec = pose.get("time", 3)
        goal.trajectory.points = [point]

        self.client.wait_for_server()
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_sent_callback)

    def goal_sent_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            # Goal rejected
            self.get_logger().error("Goal rejected!")
            rclpy.shutdown()
            return
        goal_handle.get_result_async().add_done_callback(self.goal_done_callback)

    def goal_done_callback(self, future):
        result = future.result().result
        # Reached target pose
        self.get_logger().info(f"Reached! error_code={result.error_code}")

        input(">>> Click Take Sample in easy_handeye panel, then press Enter to continue... ")

        self.current += 1
        self.next_pose()


def main():
    rclpy.init()

    if len(sys.argv) < 2:
        print("Usage: python3 auto_sample.py <poses.yaml>")
        print("YAML format example:")
        print("  - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]")
        print("    time: 3")
        sys.exit(1)

    with open(sys.argv[1]) as f:
        poses = yaml.safe_load(f)

    sampler = AutoSampler(poses)
    sampler.next_pose()
    rclpy.spin(sampler)


if __name__ == "__main__":
    main()
