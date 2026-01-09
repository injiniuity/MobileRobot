#!/usr/bin/env python
import numpy as np
import cv2
import cv2.aruco as aruco  # solved Error: $ pip install opencv-contrib-python
from flask import Flask, Response, render_template_string
from libcamera import Transform
from picamera2 import Picamera2


app = Flask(__name__)

_frame_count = 0
_last_log_frame = -999
_last_detection_frame = -999
_last_corners = None
_last_ids = None

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


def _draw_aruco(frame):
    global _frame_count, _last_log_frame, _last_detection_frame, _last_corners, _last_ids
    _frame_count += 1
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    height, width = frame.shape[:2]

    if hasattr(aruco, "DetectorParameters_create"):
        parameters = aruco.DetectorParameters_create()
    else:
        parameters = aruco.DetectorParameters()
    parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 23
    parameters.adaptiveThreshWinSizeStep = 10
    parameters.adaptiveThreshConstant = 4
    parameters.minMarkerPerimeterRate = 0.02
    parameters.maxMarkerPerimeterRate = 4.0
    parameters.polygonalApproxAccuracyRate = 0.02
    parameters.minCornerDistanceRate = 0.03
    parameters.minDistanceToBorder = 2
    parameters.cornerRefinementWinSize = 7
    parameters.cornerRefinementMaxIterations = 70
    parameters.cornerRefinementMinAccuracy = 0.005
    parameters.errorCorrectionRate = 0.8

    dict_name = "4X4_1000"
    dict_id = aruco.DICT_4X4_1000
    if hasattr(aruco, "Dictionary_get"):
        aruco_dict = aruco.Dictionary_get(dict_id)
    else:
        aruco_dict = aruco.getPredefinedDictionary(dict_id)
    if hasattr(aruco, "detectMarkers"):
        corners, ids, rejected = aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters
        )
    else:
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, rejected = detector.detectMarkers(gray)

    should_log = _frame_count - _last_log_frame >= 30
    if should_log:
        _last_log_frame = _frame_count
        if ids is not None and len(ids) > 0:
            print("DICT:", dict_name, "ids:", ids.flatten().tolist())
        else:
            print("No match. ids: None")

    hold_frames = 10
    if ids is None or len(ids) == 0:
        if _last_corners is not None and (_frame_count - _last_detection_frame) <= hold_frames:
            corners = _last_corners
            ids = _last_ids
        else:
            if should_log:
                print("Marker undetected")
                print("frame shape:", frame.shape)
            return frame
    else:
        _last_detection_frame = _frame_count
        _last_corners = corners
        _last_ids = ids

    for i in range(0, len(ids)):
        rvec, tvec = _estimate_pose_single_marker(
            corners[i], 0.03, matrix_coefficients, distortion_coefficients
        )
        if rvec is None or tvec is None:
            continue
        (rvec - tvec).any()
        aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.05)
        tvec_flat = np.array(tvec).reshape(-1)
        if tvec_flat.size != 3:
            continue
        dx, dy, dz = tvec_flat
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
        if should_log:
            print("Marker detected")
            print("marker id:", int(ids[i][0]))
            print("rvec:", rvec)
            print("tvec:", tvec)
            print("width:", width)
            print("height:", height)

    return frame


def generate_frames():
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)},
        transform=Transform(vflip=1),
    )
    picam2.configure(config)
    picam2.start()
    try:
        while True:
            frame = picam2.capture_array()
            frame = _draw_aruco(frame)
            ok, buffer = cv2.imencode(".jpg", frame)
            if not ok:
                continue
            frame_bytes = buffer.tobytes()
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
            )
    finally:
        picam2.stop()
        picam2.close()


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
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
