"""HSV traffic-light color detection (BML1 Camera/OpenCV lecture).

Pipeline: Camera frame -> HSV -> red/green masks -> pixel-area count
         -> STOP / GO / UNKNOWN signal.

Run: python hsv.py     (quit with Ctrl+C)
"""

import time

import cv2
import numpy as np
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()
time.sleep(1)

# Red wraps across H=0/179, so it needs two ranges.
red_lower_1 = np.array([0, 80, 50])
red_upper_1 = np.array([25, 255, 255])
red_lower_2 = np.array([170, 80, 50])
red_upper_2 = np.array([179, 255, 255])

green_lower = np.array([40, 80, 80])
green_upper = np.array([85, 255, 255])

min_area = 500

print("[INFO] Starting HSV traffic light detection...")

while True:
    frame = picam2.capture_array()

    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    red_mask_1 = cv2.inRange(hsv, red_lower_1, red_upper_1)
    red_mask_2 = cv2.inRange(hsv, red_lower_2, red_upper_2)
    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

    green_mask = cv2.inRange(hsv, green_lower, green_upper)

    red_area = cv2.countNonZero(red_mask)
    green_area = cv2.countNonZero(green_mask)

    if red_area > min_area and red_area > green_area:
        signal = "STOP"
    elif green_area > min_area and green_area > red_area:
        signal = "GO"
    else:
        signal = "UNKNOWN"

    print(f"[SIGNAL] {signal} | red_area={red_area}, green_area={green_area}")

    time.sleep(0.2)
