#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class CalibPublisher(Node):
    def __init__(self, name, parent_frame, child_frame):
        super().__init__("calib_publisher")

        calib_path = os.path.expanduser(
            f"~/.ros2/easy_handeye2/calibrations/{name}.calib"
        )
        if not os.path.exists(calib_path):
            self.get_logger().error(f"Calibration file not found at: {calib_path}")
            raise FileNotFoundError(calib_path)

        with open(calib_path) as f:
            data = yaml.safe_load(f)

        t = data["transform"]["translation"]
        r = data["transform"]["rotation"]

        ts = TransformStamped()
        ts.header.frame_id = parent_frame
        ts.child_frame_id = child_frame
        ts.transform.translation.x = t["x"]
        ts.transform.translation.y = t["y"]
        ts.transform.translation.z = t["z"]
        ts.transform.rotation.x = r["x"]
        ts.transform.rotation.y = r["y"]
        ts.transform.rotation.z = r["z"]
        ts.transform.rotation.w = r["w"]

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(ts)

        self.get_logger().info(
            f"Published static TF: {parent_frame} → {child_frame}  "
            f"t=({t['x']:.3f},{t['y']:.3f},{t['z']:.3f})"
        )


def main():
    import argparse
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", default="openarm_right_eob")
    parser.add_argument("--parent-frame", default="world")
    parser.add_argument("--child-frame", default="tr_base")
    args = parser.parse_args()

    node = CalibPublisher(args.name, args.parent_frame, args.child_frame)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
