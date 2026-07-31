#!/usr/bin/env python3
"""Safe manual control for ForwardCommandController with max_delta clipping.

Usage:
  python3 send_forward_cmd.py 0.0 0.0 0.0 0.0 0.0 0.0 0.0
  python3 send_forward_cmd.py --max-delta 0.02 0.0 0.0 0.0 0.0 0.0 0.0
"""

import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

RIGHT_JOINTS = [
    "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
    "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
    "openarm_right_joint7",
]
TOPIC = "/right_forward_position_controller/commands"
MAX_DELTA = 0.03
STEP_INTERVAL = 0.03


def main():
    max_delta = MAX_DELTA
    args = sys.argv[1:]

    if "--max-delta" in args:
        idx = args.index("--max-delta")
        max_delta = float(args[idx + 1])
        args = args[:idx] + args[idx + 2:]

    if len(args) != 7:
        print(f"Usage: python3 send_forward_cmd.py [--max-delta {MAX_DELTA}] j1 j2 j3 j4 j5 j6 j7")
        sys.exit(1)

    target = np.array([float(a) for a in args], dtype=np.float64)

    rclpy.init()
    node = rclpy.create_node("send_forward_cmd")
    pub = node.create_publisher(Float64MultiArray, TOPIC, 10)

    current = None

    def js_cb(msg):
        nonlocal current
        n2i = {n: i for i, n in enumerate(msg.name)}
        pos = []
        for jn in RIGHT_JOINTS:
            if jn in n2i:
                pos.append(msg.position[n2i[jn]])
            else:
                return
        current = np.array(pos, dtype=np.float64)

    sub = node.create_subscription(JointState, "/joint_states", js_cb, 10)

    print("Waiting for /joint_states ...", flush=True)
    while current is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"Current: {np.round(current, 4)}")
    print(f"Target : {np.round(target, 4)}")
    print(f"Max delta/step: {max_delta} rad\n")

    while True:
        delta = target - current
        max_d = np.max(np.abs(delta))
        if max_d < 0.005:
            print(f"Done! Final: {np.round(current, 4)}")
            break

        if max_d > max_delta:
            delta = delta * (max_delta / max_d)

        current = current + delta

        msg = Float64MultiArray()
        msg.data = current.tolist()
        pub.publish(msg)
        print(f"  -> {np.round(current, 4)}")

        time.sleep(STEP_INTERVAL)
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
