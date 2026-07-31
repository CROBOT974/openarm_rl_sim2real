#!/usr/bin/env python3
"""RL policy inference node — ForwardCommandController with max_delta clipping.

Pipeline:
  /joint_states → RL inference → max_delta clipping → ForwardCommandController → robot

Usage:
  ros2 launch openarm_sim2real sim2real_forward_cmd.launch.py \
    model_path:=/path/to/policy.pt
"""

import math
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros

DEVICE = "cpu"

RIGHT_JOINT_NAMES = [
    "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
    "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
    "openarm_right_joint7",
]
NUM_JOINTS = len(RIGHT_JOINT_NAMES)
REAL_JOINT_NAMES = RIGHT_JOINT_NAMES

FWD_CMD_TOPIC = "/right_forward_position_controller/commands"
ACTION_SCALE = 0.5
INFERENCE_RATE = 60.0
DECIMATION = 2
MAX_DELTA_PER_STEP = 0.06
EE_DIST_THRESHOLD = 0.02

DEFAULT_EE_TARGET = [0.225, -0.20, 0.35, 0.0, 3 * math.pi / 2, math.pi]


def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return np.array([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                     cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy], dtype=np.float32)


class RLPolicyNode4(Node):
    """Policy node with ForwardCommandController + max_delta clipping."""

    def __init__(self):
        super().__init__("rl_policy_node")

        self.declare_parameter("model_path", "")
        model_path = self.get_parameter("model_path").get_parameter_value().string_value

        self.get_logger().info(f"Loading policy from {model_path} ...")
        self._model = torch.jit.load(model_path, map_location=DEVICE)
        self._model.eval()
        self.get_logger().info("Policy loaded.")

        self._joint_pos = np.zeros(NUM_JOINTS, dtype=np.float32)
        self._joint_vel = np.zeros(NUM_JOINTS, dtype=np.float32)
        self._joint_received = False

        default_quat = euler_to_quat(*DEFAULT_EE_TARGET[3:])
        self._ee_target = np.concatenate([
            np.array(DEFAULT_EE_TARGET[:3], dtype=np.float32), default_quat])

        self._action = np.zeros(NUM_JOINTS, dtype=np.float32)
        self._previous_action = np.zeros(NUM_JOINTS, dtype=np.float32)
        self._policy_counter = 0

        self._js_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10)

        self._ee_sub = self.create_subscription(
            Float64MultiArray, "/right_ee_target", self._ee_target_callback, 10)
        self._ee_pub = self.create_publisher(
            Float64MultiArray, "/right_ee_target_viz", 10)

        self._fwd_pub = self.create_publisher(
            Float64MultiArray, FWD_CMD_TOPIC, 10)

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._timer = self.create_timer(1.0 / INFERENCE_RATE, self._inference_step)

        self.get_logger().info(
            f"Ready @ {INFERENCE_RATE} Hz, decimation={DECIMATION}, "
            f"max_delta={MAX_DELTA_PER_STEP} rad/step, device={DEVICE}")

    def _joint_state_callback(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        matched = 0
        for i in range(NUM_JOINTS):
            if REAL_JOINT_NAMES[i] in name_to_idx:
                idx = name_to_idx[REAL_JOINT_NAMES[i]]
                self._joint_pos[i] = msg.position[idx]
                if idx < len(msg.velocity):
                    self._joint_vel[i] = msg.velocity[idx]
                matched += 1
        if matched == NUM_JOINTS:
            self._joint_received = True

    def _ee_target_callback(self, msg: Float64MultiArray):
        data = msg.data
        if len(data) == 6:
            quat = euler_to_quat(data[3], data[4], data[5])
            self._ee_target = np.concatenate([
                np.array(data[:3], dtype=np.float32), quat.astype(np.float32)])
        elif len(data) == 7:
            self._ee_target = np.array(data[:7], dtype=np.float32)
        self._ee_pub.publish(Float64MultiArray(data=data[:3]))

    def _inference_step(self):
        if not self._joint_received:
            return

        if self._policy_counter % DECIMATION == 0:
            # Stop if EE is close enough to target
            try:
                t = self._tf_buffer.lookup_transform(
                    "openarm_body_link0", "openarm_right_hand",
                    rclpy.time.Time())
                ee_pos = np.array([t.transform.translation.x,
                                   t.transform.translation.y,
                                   t.transform.translation.z])
                ee_dist = np.linalg.norm(ee_pos - self._ee_target[:3])
                if ee_dist < EE_DIST_THRESHOLD:
                    self._policy_counter += 1
                    return
            except Exception:
                pass

            obs = np.concatenate([
                self._joint_pos, self._joint_vel, self._ee_target,
                self._previous_action,
            ]).astype(np.float32)

            obs_tensor = torch.from_numpy(obs).unsqueeze(0).to(DEVICE)
            with torch.no_grad():
                raw_action = self._model(obs_tensor).cpu().numpy().squeeze(0)
            self._action = raw_action.astype(np.float32)
            self._previous_action = self._action.copy()

            rl_target = (ACTION_SCALE * self._action).astype(np.float64)

            current = self._joint_pos.astype(np.float64)
            delta = rl_target - current
            max_delta = np.max(np.abs(delta))
            if max_delta > MAX_DELTA_PER_STEP:
                delta = delta * (MAX_DELTA_PER_STEP / max_delta)

            msg = Float64MultiArray()
            msg.data = (current + delta).tolist()
            self._fwd_pub.publish(msg)

        self._publish_tf()
        self._ee_pub.publish(Float64MultiArray(data=self._ee_target[:3].tolist()))
        self._policy_counter += 1

    def _publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "openarm_body_link0"
        t.child_frame_id = "rl_target"
        t.transform.translation.x = float(self._ee_target[0])
        t.transform.translation.y = float(self._ee_target[1])
        t.transform.translation.z = float(self._ee_target[2])
        t.transform.rotation.w = float(self._ee_target[3])
        t.transform.rotation.x = float(self._ee_target[4])
        t.transform.rotation.y = float(self._ee_target[5])
        t.transform.rotation.z = float(self._ee_target[6])
        self._tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = RLPolicyNode4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
