#!/usr/bin/env python3
"""RL policy inference node — multi-target cycle.

Cycles through a list of EE targets, staying at each for a fixed duration.
Pipeline: /joint_states → RL inference → max_delta clipping → ForwardCommandController → robot

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

TARGET_CYCLE = [
    [0.25, -0.18, 0.32, 0.0, 3 * math.pi / 2, math.pi],
    [0.20, -0.20, 0.40, 0.0, 3 * math.pi / 2, math.pi],
    [0.28, -0.16, 0.36, 0.0, 3 * math.pi / 2, math.pi],
]
TARGET_STAY_SECONDS = 5.0


def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return np.array([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                     cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy], dtype=np.float32)


class RLPolicyNodeCycle(Node):
    """Policy node with multi-target cycling."""

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

        self._action = np.zeros(NUM_JOINTS, dtype=np.float32)
        self._previous_action = np.zeros(NUM_JOINTS, dtype=np.float32)
        self._policy_counter = 0

        self._cycle_idx = 0
        self._arrived_time = None
        self._set_ee_from_euler(TARGET_CYCLE[0])

        self._js_sub = self.create_subscription(JointState, "/joint_states", self._cb_joint, 10)
        self._fwd_pub = self.create_publisher(Float64MultiArray, FWD_CMD_TOPIC, 10)
        self._ee_pub = self.create_publisher(Float64MultiArray, "/right_ee_target_viz", 10)

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._timer = self.create_timer(1.0 / INFERENCE_RATE, self._inference_step)

        self.get_logger().info(
            f"Ready @ {INFERENCE_RATE} Hz, decimation={DECIMATION}, "
            f"max_delta={MAX_DELTA_PER_STEP} rad/step, "
            f"cycling {len(TARGET_CYCLE)} targets, stay={TARGET_STAY_SECONDS}s each")

    def _set_ee_from_euler(self, euler_target):
        quat = euler_to_quat(euler_target[3], euler_target[4], euler_target[5])
        self._ee_target = np.concatenate([
            np.array(euler_target[:3], dtype=np.float32), quat.astype(np.float32)])

    def _switch_to_next_target(self):
        self._cycle_idx = (self._cycle_idx + 1) % len(TARGET_CYCLE)
        t = TARGET_CYCLE[self._cycle_idx]
        self._set_ee_from_euler(t)
        self._arrived_time = None
        self.get_logger().info(
            f"-> Target {self._cycle_idx}: "
            f"pos=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f})")

    def _cb_joint(self, msg: JointState):
        n2i = {n: i for i, n in enumerate(msg.name)}
        m = 0
        for i in range(NUM_JOINTS):
            if REAL_JOINT_NAMES[i] in n2i:
                idx = n2i[REAL_JOINT_NAMES[i]]
                self._joint_pos[i] = msg.position[idx]
                if idx < len(msg.velocity):
                    self._joint_vel[i] = msg.velocity[idx]
                m += 1
        if m == NUM_JOINTS:
            self._joint_received = True

    def _inference_step(self):
        if not self._joint_received:
            return

        if self._policy_counter % DECIMATION == 0:
            try:
                t = self._tf_buffer.lookup_transform(
                    "openarm_body_link0", "openarm_right_hand", rclpy.time.Time())
                ee_pos = np.array([
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z])
                ee_dist = np.linalg.norm(ee_pos - self._ee_target[:3])

                if ee_dist < EE_DIST_THRESHOLD:
                    now = self.get_clock().now()
                    if self._arrived_time is None:
                        self._arrived_time = now
                    elif (now - self._arrived_time).nanoseconds * 1e-9 >= TARGET_STAY_SECONDS:
                        self._switch_to_next_target()
                    self._policy_counter += 1
                    return
                else:
                    self._arrived_time = None
            except Exception:
                pass

            obs = np.concatenate([
                self._joint_pos, self._joint_vel, self._ee_target,
                self._previous_action,
            ]).astype(np.float32)

            obs_t = torch.from_numpy(obs).unsqueeze(0).to(DEVICE)
            with torch.no_grad():
                raw = self._model(obs_t).cpu().numpy().squeeze(0)
            self._action = raw.astype(np.float32)
            self._previous_action = self._action.copy()

            rl_target = (ACTION_SCALE * self._action).astype(np.float64)
            current = self._joint_pos.astype(np.float64)
            delta = rl_target - current
            max_d = np.max(np.abs(delta))
            if max_d > MAX_DELTA_PER_STEP:
                delta = delta * (MAX_DELTA_PER_STEP / max_d)

            msg = Float64MultiArray()
            msg.data = (current + delta).tolist()
            self._fwd_pub.publish(msg)

        self._publish_tf()
        self._ee_pub.publish(Float64MultiArray(data=self._ee_target[:3].tolist()))
        self._policy_counter += 1

    def _publish_tf(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "openarm_body_link0"
        tf.child_frame_id = "rl_target"
        tf.transform.translation.x = float(self._ee_target[0])
        tf.transform.translation.y = float(self._ee_target[1])
        tf.transform.translation.z = float(self._ee_target[2])
        tf.transform.rotation.w = float(self._ee_target[3])
        tf.transform.rotation.x = float(self._ee_target[4])
        tf.transform.rotation.y = float(self._ee_target[5])
        tf.transform.rotation.z = float(self._ee_target[6])
        self._tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = RLPolicyNodeCycle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
