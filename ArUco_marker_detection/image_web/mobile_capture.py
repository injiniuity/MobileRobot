import threading
import time

import cv2
from flask import Flask, Response, render_template_string
from libcamera import Transform
from picamera2 import Picamera2

#25mm
app = Flask(__name__)

_frame = None
_frame_lock = threading.Lock()
_stop_event = threading.Event()
_img_counter = 0


def _capture_loop():
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)},
        transform=Transform(vflip=1),
    )
    picam2.configure(config)
    picam2.start()
    try:
        while not _stop_event.is_set():
            frame = picam2.capture_array()
            with _frame_lock:
                global _frame
                _frame = frame.copy()
    finally:
        picam2.stop()
        picam2.close()


def _frame_generator():
    while not _stop_event.is_set():
        with _frame_lock:
            current = _frame.copy() if _frame is not None else None
        if current is None:
            time.sleep(0.05)
            continue
        ok, buffer = cv2.imencode(".jpg", current)
        if not ok:
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
            <title>Mobile Capture</title>
          </head>
          <body>
            <h1>Mobile Capture</h1>
            <img src="/stream" style="max-width: 100%; height: auto;" />
            <div style="margin-top: 12px;">
              <button onclick="fetch('/capture').then(r => r.text()).then(t => alert(t));">
                Capture
              </button>
            </div>
          </body>
        </html>
        """
    )


@app.route("/stream")
def stream():
    return Response(
        _frame_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/capture")
def capture():
    global _img_counter
    with _frame_lock:
        current = _frame.copy() if _frame is not None else None
    if current is None:
        return "No frame available.", 503
    img_name = "/home/pinky/pinky_pro/Jini/ArUco_marker_detection/aruco/src/image_web/captures/mobile_frame_{:03d}.png".format(_img_counter)
    cv2.imwrite(img_name, current)
    _img_counter += 1
    return "{} written!".format(img_name)


if __name__ == "__main__":
    _stop_event.clear()
    threading.Thread(target=_capture_loop, daemon=True).start()
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True, use_reloader=False)
