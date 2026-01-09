#!/usr/bin/env python
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import cv2.aruco as aruco
from flask import Flask, Response, render_template_string
from libcamera import Transform
from picamera2 import Picamera2


app = Flask(__name__)

_frame_lock = threading.Lock()
_latest_frame = None

DICT_NAME = "4X4_1000"
DICT_ID = aruco.DICT_4X4_1000

TARGET_Z_M = 0.20
TOL_Z_M = 0.01
TOL_X_PX = 20
KP_Z = 0.8
KP_X = 0.004
MAX_LIN = 0.15
KP_YAW = 2.0
MAX_YAW = 0.6
X_SIGN = 1.0
Z_SIGN = -1.0

matrix_coefficients = np.array(
    [
        [563.2946110095496, 0.0, 303.9646991977407],
        [0.0, 560.826815954084, 225.5769044445312],
        [0.0, 0.0, 1.0],
    ]
)
distortion_coefficients = np.array(
    [
        0.20050998993862901,
        -0.9199774166653199,
        -0.002612006745063098,
        -0.011228400891975006,
        1.7447997750399993,
    ]
)


def _estimate_pose_single_marker(corners, marker_length, camera_matrix, dist_coeffs):
    if hasattr(aruco, "estimatePoseSingleMarkers"):
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, marker_length, camera_matrix, dist_coeffs
        )
        return rvecs, tvecs

    half = marker_length / 2.0
    objp = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float32,
    )
    imgp = corners.reshape(-1, 2).astype(np.float32)
    ok, rvec, tvec = cv2.solvePnP(
        objp, imgp, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not ok:
        return None, None
    return np.array([rvec], dtype=np.float32), np.array([tvec], dtype=np.float32)


def _get_detector_parameters():
    if hasattr(aruco, "DetectorParameters_create"):
        params = aruco.DetectorParameters_create()
    else:
        params = aruco.DetectorParameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 10
    params.adaptiveThreshConstant = 4
    params.minMarkerPerimeterRate = 0.02
    params.maxMarkerPerimeterRate = 4.0
    params.polygonalApproxAccuracyRate = 0.02
    params.minCornerDistanceRate = 0.03
    params.minDistanceToBorder = 2
    params.cornerRefinementWinSize = 7
    params.cornerRefinementMaxIterations = 70
    params.cornerRefinementMinAccuracy = 0.005
    params.errorCorrectionRate = 0.8
    return params


def _get_dictionary():
    if hasattr(aruco, "Dictionary_get"):
        return aruco.Dictionary_get(DICT_ID)
    return aruco.getPredefinedDictionary(DICT_ID)


def _detect_pose(frame, aruco_dict, parameters):
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    if hasattr(aruco, "detectMarkers"):
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    else:
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return None, None, None
    rvec, tvec = _estimate_pose_single_marker(
        corners[0], 0.03, matrix_coefficients, distortion_coefficients
    )
    if rvec is None or tvec is None:
        return None, None, None
    tvec_flat = np.array(tvec).reshape(-1)
    if tvec_flat.size != 3:
        return None, None, None
    return corners, rvec, tvec_flat


def _center_error_px(corners):
    pts = corners[0].reshape(-1, 2)
    center = pts.mean(axis=0)
    return float(center[0])


def _set_latest_frame(frame):
    global _latest_frame
    with _frame_lock:
        _latest_frame = frame


def _get_latest_frame():
    with _frame_lock:
        if _latest_frame is None:
            return None
        return _latest_frame.copy()


def _stream_frames():
    while True:
        frame = _get_latest_frame()
        if frame is None:
            time.sleep(0.05)
            continue
        ok, buffer = cv2.imencode(".jpg", frame)
        if not ok:
            time.sleep(0.05)
            continue
        frame_bytes = buffer.tobytes()
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
        )


@app.route("/")
def index():
    return render_template_string(
        """
        <!doctype html>
        <html>
          <head>
            <meta charset="utf-8">
            <title>ArUco Stream</title>
          </head>
          <body>
            <h1>ArUco Stream</h1>
            <img src="/stream" style="max-width: 100%; height: auto;" />
          </body>
        </html>
        """
    )


@app.route("/stream")
def stream():
    return Response(
        _stream_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


class PrecisionController(Node):
    def __init__(self):
        super().__init__("precision_aruco_controller")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def run(self):
        aruco_dict = _get_dictionary()
        parameters = _get_detector_parameters()
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (640, 480)},
            transform=Transform(vflip=1),
        )
        picam2.configure(config)
        picam2.start()
        server_thread = threading.Thread(
            target=app.run,
            kwargs={"host": "0.0.0.0", "port": 5000, "debug": False, "threaded": True},
            daemon=True,
        )
        server_thread.start()
        try:
            while rclpy.ok():
                frame = picam2.capture_array()
                corners, rvec, tvec = _detect_pose(frame, aruco_dict, parameters)
                if tvec is None:
                    _set_latest_frame(frame)
                    self.pub.publish(Twist())
                    time.sleep(0.05)
                    continue
                aruco.drawDetectedMarkers(frame, corners)
                cv2.drawFrameAxes(
                    frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.05
                )
                dx, dy, dz = tvec
                center_x = _center_error_px(corners)
                err_x_px = center_x - (frame.shape[1] / 2.0)
                err_z = Z_SIGN * (dz - TARGET_Z_M)
                cv2.putText(
                    frame,
                    "dx={:.3f} dy={:.3f} dz={:.3f} m".format(dx, dy, dz),
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    "center_px={:.1f} move_z={:.3f}".format(err_x_px, err_z),
                    (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                _set_latest_frame(frame)
                msg = Twist()
                if abs(err_x_px) > TOL_X_PX:
                    msg.angular.z = float(np.clip(KP_X * err_x_px, -MAX_YAW, MAX_YAW))
                else:
                    msg.linear.x = float(np.clip(KP_Z * err_z, -MAX_LIN, MAX_LIN))
                self.pub.publish(msg)
                if abs(err_x_px) <= TOL_X_PX and abs(err_z) <= TOL_Z_M:
                    break
                time.sleep(0.05)
        finally:
            self.pub.publish(Twist())
            picam2.stop()
            picam2.close()


def main():
    rclpy.init()
    node = PrecisionController()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
