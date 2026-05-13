"""HSV + circularity check for traffic-light detection (BML1 Camera lecture).

Improves over plain HSV by requiring color blobs to be roughly circular,
filtering out red shirts / brake lights / etc.

Run: python hsv_circle.py     (quit with Ctrl+C)
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

red_lower_1 = np.array([0, 80, 50])
red_upper_1 = np.array([25, 255, 255])
red_lower_2 = np.array([170, 80, 50])
red_upper_2 = np.array([179, 255, 255])

green_lower = np.array([40, 80, 80])
green_upper = np.array([85, 255, 255])

min_area = 300
min_circularity = 0.65
min_radius = 8
max_radius = 120


def get_circular_area(mask):
    """Sum the area of contours that pass the circularity + radius test."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    circular_area = 0

    for contour in contours:
        area = cv2.contourArea(contour)

        if area < min_area:
            continue

        perimeter = cv2.arcLength(contour, True)

        if perimeter == 0:
            continue

        circularity = 4 * np.pi * area / (perimeter * perimeter)

        (_x, _y), radius = cv2.minEnclosingCircle(contour)

        if circularity >= min_circularity and min_radius <= radius <= max_radius:
            circular_area += area

    return circular_area


print("[INFO] Starting HSV circle traffic light detection...")

while True:
    frame = picam2.capture_array()

    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    red_mask_1 = cv2.inRange(hsv, red_lower_1, red_upper_1)
    red_mask_2 = cv2.inRange(hsv, red_lower_2, red_upper_2)
    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

    green_mask = cv2.inRange(hsv, green_lower, green_upper)

    red_circle_area = get_circular_area(red_mask)
    green_circle_area = get_circular_area(green_mask)

    if red_circle_area > min_area and red_circle_area > green_circle_area:
        signal = "STOP"
    elif green_circle_area > min_area and green_circle_area > red_circle_area:
        signal = "GO"
    else:
        signal = "UNKNOWN"

    print(
        f"[SIGNAL] {signal} | red_circle_area={red_circle_area}, "
        f"green_circle_area={green_circle_area}"
    )

    time.sleep(0.2)
