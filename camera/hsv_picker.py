"""On-site HSV calibration tool — click a pixel, see its H/S/V.

Use this when red/green thresholds in hsv_circle.py or yolo_hsv.py
need tuning under your actual lighting + actual traffic-light model.

How to use:
  1. Run this script.
  2. Hold up the traffic light (RED on).
  3. Click on the red bulb in the preview window — its HSV prints to terminal.
  4. Repeat for green.
  5. Also click UNLIT bulbs and the case/background to learn what to reject.
  6. Update the `*_lower / *_upper` arrays in hsv_circle.py / yolo_hsv.py
     so each LIT color's measured H/S/V falls inside its range while UNLIT
     and background values stay outside.

Quit: 'q' in the preview window.
"""

import time

import cv2
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"  # numpy order is BGR
picam2.configure("preview")
picam2.start()
time.sleep(1)

latest_frame = {"bgr": None, "hsv": None}


def on_click(event, x, y, flags, _param):
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    hsv = latest_frame["hsv"]
    bgr = latest_frame["bgr"]
    if hsv is None:
        return
    h, s, v = hsv[y, x]
    b, g, r = bgr[y, x]
    print(f"[PICK] xy=({x:>3},{y:>3})  HSV=({h:>3},{s:>3},{v:>3})  BGR=({b:>3},{g:>3},{r:>3})")


cv2.namedWindow("HSV Picker")
cv2.setMouseCallback("HSV Picker", on_click)

print("[INFO] Click anywhere in the window to read HSV. Press 'q' to quit.")

try:
    while True:
        frame = picam2.capture_array()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        latest_frame["bgr"] = frame
        latest_frame["hsv"] = hsv

        cv2.imshow("HSV Picker", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    cv2.destroyAllWindows()
    picam2.stop()
