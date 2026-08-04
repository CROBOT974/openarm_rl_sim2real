#!/usr/bin/env python3

import sys
import cv2
import numpy as np
import ctypes
from ctypes import cast, POINTER

MVS_DIR = "/opt/MVS/Samples/64/Python"
sys.path.append(MVS_DIR)
from MvImport.MvCameraControl_class import *

# ═══════════════════════════════════════════════════════════
# Adjustable parameters
# ═══════════════════════════════════════════════════════════
CHECKERBOARD = (9, 6)      # Number of internal corners in chessboard
SQUARE_SIZE  = 0.023       # Physical size of each square, in meters

# Camera settings
EXPOSURE_TIME = 20000.0
GAIN          = 10.0
GAMMA         = 0.6


def main():
    dl = MV_CC_DEVICE_INFO_LIST()
    MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE, dl)
    if dl.nDeviceNum == 0:
        print("No USB industrial camera found!")
        sys.exit(1)

    cam = MvCamera()
    st = cast(dl.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents
    cam.MV_CC_CreateHandle(st)
    cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)

    cam.MV_CC_SetIntValue("OffsetX", 0)
    cam.MV_CC_SetIntValue("OffsetY", 0)
    cam.MV_CC_SetEnumValueByString("ExposureAuto", "Off")
    cam.MV_CC_SetEnumValueByString("GainAuto", "Off")
    cam.MV_CC_SetFloatValue("ExposureTime", EXPOSURE_TIME)
    cam.MV_CC_SetFloatValue("Gain", GAIN)
    cam.MV_CC_SetBoolValue("GammaEnable", True)
    cam.MV_CC_SetFloatValue("Gamma", GAMMA)

    p = MVCC_INTVALUE()
    cam.MV_CC_GetIntValue("PayloadSize", p)
    buf = (ctypes.c_ubyte * p.nCurValue)()
    fi = MV_FRAME_OUT_INFO_EX()
    cam.MV_CC_StartGrabbing()

    # 3D physical point template for chessboard calibration
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    obj_points = []   # 3D
    img_points = []   # 2D

    print(f"Chessboard {CHECKERBOARD[0]}x{CHECKERBOARD[1]}，格子 {SQUARE_SIZE*1000:.0f}mm")
    print("Press c to capture, press s to solve, press q to exit")

    while True:
        if cam.MV_CC_GetOneFrameTimeout(byref(buf), p.nCurValue, fi, 200) != 0:
            cv2.waitKey(1)
            continue

        raw = np.frombuffer(buf, np.uint8, count=int(fi.nFrameLen))
        raw = raw.reshape(fi.nHeight, fi.nWidth)
        gray = cv2.cvtColor(raw, cv2.COLOR_BAYER_GR2GRAY)
        color = cv2.cvtColor(raw, cv2.COLOR_BAYER_GR2RGB)

        found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        display = color.copy()

        # Visual status prompt
        if found:
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            cv2.drawChessboardCorners(display, CHECKERBOARD, corners, found)
            cv2.putText(display, "READY - Press C to capture",
                        (20, display.shape[0]-30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
        else:
            cv2.putText(display, "NO BOARD - adjust angle/distance",
                        (20, display.shape[0]-30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

        cv2.putText(display, f"Collected: {len(obj_points)} | [c]apture [s]olve [q]uit",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.imshow("Camera Calib", cv2.resize(display, (960, 540)))

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        elif key == ord("c"):
            if not found:
                print(f"\rChessboard not detected! (Total {len(obj_points)} frames)", end="", flush=True)
                continue
            obj_points.append(objp)
            img_points.append(corners)
            print(f"\n✅ Collected {len(obj_points)} frames")
        elif key == ord("s"):
            if len(obj_points) < 10:
                print(f"Need at least 10 frames, current {len(obj_points)} frames")
                continue

            print(f"Using {len(obj_points)} frames to solve...")
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                obj_points, img_points, gray.shape[::-1], None, None)

            fx, fy = mtx[0,0], mtx[1,1]
            cx, cy = mtx[0,2], mtx[1,2]

            print("\n" + "="*60)
            print(f"Reprojection error: {ret:.4f} px  ({'✅ Good' if ret < 0.5 else '⚠️ Strongly available' if ret < 1.0 else '❌ Too bad, resample'})")
            print(f"\nCamera matrix (copy to aruco_pose_node.py with --fx --fy --cx --cy):")
            print(f"  --fx {fx:.2f} --fy {fy:.2f} --cx {cx:.2f} --cy {cy:.2f}")
            print(f"\nDistortion coefficients:")
            print(f"  {dist.ravel()}")
            print("="*60)

            np.savez("camera_calib.npz", mtx=mtx, dist=dist)
            print("Saved to camera_calib.npz")

    cam.MV_CC_StopGrabbing()
    cam.MV_CC_CloseDevice()
    cam.MV_CC_DestroyHandle()
    cv2.destroyAllWindows()
    print("Exited")


if __name__ == "__main__":
    main()
