#!/usr/bin/env python3

import sys
import time
import argparse
import threading

import cv2
import numpy as np
import ctypes
from ctypes import cast, POINTER, byref

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

MVS_PYTHON_DIR = "/opt/MVS/Samples/64/Python"
if MVS_PYTHON_DIR not in sys.path:
    sys.path.append(MVS_PYTHON_DIR)

from MvImport.MvCameraControl_class import (  # type: ignore
    MvCamera,
    MV_CC_DEVICE_INFO_LIST,
    MV_CC_DEVICE_INFO,
    MV_USB_DEVICE,
    MV_ACCESS_Exclusive,
    MVCC_INTVALUE,
    MV_FRAME_OUT_INFO_EX,
)

# Camera intrinsic parameters （Alter after calibration）
DEFAULT_CAMERA_MATRIX = np.array(
    [[1783.1, 0.0, 2031.6], [0.0, 1792.7, 1185.5], [0.0, 0.0, 1.0]],
    dtype=np.float32,
)
DEFAULT_DIST_COEFFS = np.zeros(5, dtype=np.float32)


def rodrigues_to_quaternion(rvec):
    """cv2.Rodrigues rotation vector → quaternion [x, y, z, w]"""
    R, _ = cv2.Rodrigues(rvec)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return float(qx), float(qy), float(qz), float(qw)


class ArucoPoseNode(Node):
    def __init__(self, marker_id, marker_size, camera_frame, marker_frame,
                 camera_matrix, dist_coeffs):
        super().__init__("aruco_pose_node")

        self.marker_id = marker_id
        self.marker_size = marker_size
        self.camera_frame = camera_frame
        self.marker_frame = marker_frame
        self.camera_matrix = np.array(camera_matrix, dtype=np.float32)
        self.dist_coeffs = np.array(dist_coeffs, dtype=np.float32)

        half = marker_size / 2
        self._obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32)

        self.tf_broadcaster = TransformBroadcaster(self)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self.show_preview = True
        self.window_name = "ArUco Pose"

        self.cam = None
        self.data_buf = None
        self.payload_size = 0
        self._stop_event = threading.Event()
        self.get_logger().info(
            f"ArUco detector ready: marker_id={marker_id}, size={marker_size}m"
        )

    def _open_camera(self):
        device_list = MV_CC_DEVICE_INFO_LIST()
        ret = MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE, device_list)
        if ret != 0 or device_list.nDeviceNum == 0:
            self.get_logger().error("No USB industrial camera found!")
            return False

        self.cam = MvCamera()
        st_device = cast(
            device_list.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)
        ).contents
        self.cam.MV_CC_CreateHandle(st_device)
        self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)

        # 重置 ROI 偏移（prevent residual errors from previous runs）
        self.cam.MV_CC_SetIntValue("OffsetX", 0)
        self.cam.MV_CC_SetIntValue("OffsetY", 0)

        self.cam.MV_CC_SetEnumValueByString("ExposureAuto", "Off")
        self.cam.MV_CC_SetEnumValueByString("GainAuto", "Off")
        self.cam.MV_CC_SetFloatValue("ExposureTime", 20000.0)
        self.cam.MV_CC_SetFloatValue("Gain", 10.0)
        self.cam.MV_CC_SetBoolValue("GammaEnable", True)
        self.cam.MV_CC_SetFloatValue("Gamma", 0.6)

        st_param = MVCC_INTVALUE()
        self.cam.MV_CC_GetIntValue("PayloadSize", st_param)
        self.payload_size = st_param.nCurValue
        self.data_buf = (ctypes.c_ubyte * self.payload_size)()

        self.cam.MV_CC_StartGrabbing()
        self.get_logger().info("Camera started!")
        return True

    def _close_camera(self):
        if self.cam is not None:
            self.cam.MV_CC_StopGrabbing()
            self.cam.MV_CC_CloseDevice()
            self.cam.MV_CC_DestroyHandle()
            self.cam = None

    def spin(self):
        for attempt in range(3):
            if self._open_camera():
                break
            self.get_logger().warn(f"Camera open failed, retrying {attempt+1}/3...")
            time.sleep(2.0)
        else:
            self.get_logger().error("Camera open failed 3 times, node exiting. Please check USB connection, MVS is open in other program.")
            return

        if self.show_preview:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        st_frame = MV_FRAME_OUT_INFO_EX()
        last_tf_time = self.get_clock().now()

        while rclpy.ok() and not self._stop_event.is_set():
            ret = self.cam.MV_CC_GetOneFrameTimeout(
                byref(self.data_buf), self.payload_size, st_frame, 200
            )
            if ret != 0:
                if self.show_preview:
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        break
                continue

            img_raw = np.frombuffer(
                self.data_buf, count=int(st_frame.nFrameLen), dtype=np.uint8
            ).reshape((st_frame.nHeight, st_frame.nWidth))
            img_gray = cv2.cvtColor(img_raw, cv2.COLOR_BAYER_GR2GRAY)
            img_gray = self.clahe.apply(img_gray)

            if self.show_preview:
                display = cv2.cvtColor(img_raw, cv2.COLOR_BAYER_GR2RGB)

            corners, ids, _ = self.detector.detectMarkers(img_gray)

            target_corner = None
            target_rvec = None
            target_tvec = None

            if ids is not None:
                for i, mid in enumerate(ids):
                    c = corners[i]
                    cx = int(np.mean(c[0, :, 0]))
                    cy = int(np.mean(c[0, :, 1]))

                    _, rvec, tvec = cv2.solvePnP(
                        self._obj_points,
                        c,
                        self.camera_matrix,
                        self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE,
                    )

                    if mid[0] == self.marker_id:
                        target_corner = c
                        target_rvec = rvec
                        target_tvec = tvec

                    if self.show_preview:
                        color = (0, 255, 0) if mid[0] == self.marker_id else (0, 0, 255)
                        pts = c[0].astype(np.int32)
                        for j in range(4):
                            cv2.line(display, tuple(pts[j]),
                                     tuple(pts[(j+1) % 4]), color, 2)
                        cv2.circle(display, (cx, cy), 5, color, -1)
                        cv2.putText(display,
                            f"ID:{mid[0]} ({tvec[0,0]:.2f},{tvec[1,0]:.2f},{tvec[2,0]:.2f})",
                            (cx-80, cy-30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            if target_corner is None:
                if self.show_preview:
                    cv2.putText(display, "NO TARGET", (20, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            else:
                qx, qy, qz, qw = rodrigues_to_quaternion(target_rvec)

                now = self.get_clock().now().to_msg()
                ts = TransformStamped()
                ts.header.stamp = now
                ts.header.frame_id = self.camera_frame
                ts.child_frame_id = self.marker_frame
                ts.transform.translation.x = float(target_tvec[0])
                ts.transform.translation.y = float(target_tvec[1])
                ts.transform.translation.z = float(target_tvec[2])
                ts.transform.rotation.x = qx
                ts.transform.rotation.y = qy
                ts.transform.rotation.z = qz
                ts.transform.rotation.w = qw

                self.tf_broadcaster.sendTransform(ts)

                if self.show_preview:
                    cv2.putText(display,
                        f"TARGET ID={self.marker_id}  "
                        f"({target_tvec[0,0]:.3f},{target_tvec[1,0]:.3f},{target_tvec[2,0]:.3f})",
                        (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                if (self.get_clock().now() - last_tf_time).nanoseconds > 2e9:
                    self.get_logger().info(
                        f"Detected marker {self.marker_id}: "
                        f"t=({target_tvec[0,0]:.3f},{target_tvec[1,0]:.3f},{target_tvec[2,0]:.3f})"
                    )
                    last_tf_time = self.get_clock().now()

            if self.show_preview:
                cv2.imshow(self.window_name, cv2.resize(display, (960, 540)))
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break

        self._close_camera()
        if self.show_preview:
            cv2.destroyAllWindows()

    def stop(self):
        self._stop_event.set()


def main():
    rclpy.init()

    parser = argparse.ArgumentParser(description="ArUco Pose TF Publisher")
    parser.add_argument("--marker-id", type=int, default=10)
    parser.add_argument("--marker-size", type=float, default=0.05,
                        help="ArUco marker edge length, unit: meter")
    parser.add_argument("--camera-frame", type=str, default="camera_frame")
    parser.add_argument("--marker-frame", type=str, default="aruco_marker_frame")
    parser.add_argument("--fx", type=float, default=None)
    parser.add_argument("--fy", type=float, default=None)
    parser.add_argument("--cx", type=float, default=None)
    parser.add_argument("--cy", type=float, default=None)
    args = parser.parse_args()

    camera_matrix = [
        [args.fx if args.fx else float(DEFAULT_CAMERA_MATRIX[0, 0]), 0.0,
         args.cx if args.cx else float(DEFAULT_CAMERA_MATRIX[0, 2])],
        [0.0, args.fy if args.fy else float(DEFAULT_CAMERA_MATRIX[1, 1]),
         args.cy if args.cy else float(DEFAULT_CAMERA_MATRIX[1, 2])],
        [0.0, 0.0, 1.0],
    ]

    node = ArucoPoseNode(
        marker_id=args.marker_id,
        marker_size=args.marker_size,
        camera_frame=args.camera_frame,
        marker_frame=args.marker_frame,
        camera_matrix=camera_matrix,
        dist_coeffs=DEFAULT_DIST_COEFFS.tolist(),
    )

    spin_thread = threading.Thread(target=node.spin, daemon=True)
    spin_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
