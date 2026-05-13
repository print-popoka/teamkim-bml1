"""HSV + circularity + temporal smoothing for traffic-light detection.

Improvements over the lecture baseline:
  - picamera2's "RGB888" actually returns BGR-ordered ndarrays, so we use
    COLOR_BGR2HSV (not RGB2HSV) — fixes "red looks like blue" bugs.
  - ROI: only the upper 70% of the frame is searched (signals are overhead).
  - Morphological open+close to kill speckle and fill small gaps.
  - Wider red range and lower S floor — real LED reds desaturate at distance.
  - Temporal smoothing: 5-frame majority vote, avoids flickering decisions.
  - Prints raw mask areas every loop for easy on-site tuning.

Run: python camera/hsv_circle.py     (quit with Ctrl+C)
"""

import time
from collections import Counter, deque

import cv2
import numpy as np
from picamera2 import Picamera2

# Camera --------------------------------------------------------------
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"  # libcamera quirk: numpy order is BGR
picam2.configure("preview")
picam2.start()
time.sleep(1)

# HSV ranges ----------------------------------------------------------
# Tuned from printed traffic-light samples. The key is a high V (brightness)
# floor — only the LIT bulb passes, the colored-but-dark unlit lenses fall
# below the threshold. Red and yellow H ranges are kept disjoint to avoid
# the red<->yellow swap.
red_lower_1 = np.array([0, 120, 150])
red_upper_1 = np.array([8, 255, 255])
red_lower_2 = np.array([170, 120, 150])
red_upper_2 = np.array([179, 255, 255])

yellow_lower = np.array([18, 120, 170])
yellow_upper = np.array([32, 255, 255])

green_lower = np.array([40, 80, 120])
green_upper = np.array([85, 255, 255])

# Winner must beat runner-up by this factor to commit a decision.
WIN_MARGIN = 1.5

# Shape thresholds ----------------------------------------------------
min_area = 200
min_circularity = 0.55
min_radius = 6
max_radius = 140

# Temporal smoothing --------------------------------------------------
SMOOTH_WINDOW = 5
SMOOTH_MIN_VOTES = 3
history = deque(maxlen=SMOOTH_WINDOW)

# Morphology kernel ---------------------------------------------------
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def clean_mask(mask):
    """Open to remove speckle, close to fill small gaps."""
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=2)
    return mask


def get_circular_area(mask):
    """Sum the area of contours that pass circularity + radius checks."""
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


def smooth_signal(raw):
    """Return STOP/GO only if it dominates the last SMOOTH_WINDOW frames."""
    history.append(raw)
    if len(history) < SMOOTH_WINDOW:
        return raw
    top, votes = Counter(history).most_common(1)[0]
    return top if votes >= SMOOTH_MIN_VOTES else "UNKNOWN"


print("[INFO] Starting HSV circle traffic light detection...")
print("[INFO] Tune red/green HSV ranges if [DEBUG] areas look wrong under your lighting.")

try:
    while True:
        frame = picam2.capture_array()

        # ROI: upper 70% of the image (traffic lights are overhead).
        h, w = frame.shape[:2]
        roi = frame[: int(h * 0.7), :]

        # picamera2 RGB888 -> BGR in numpy, so BGR2HSV.
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        red_mask_1 = cv2.inRange(hsv, red_lower_1, red_upper_1)
        red_mask_2 = cv2.inRange(hsv, red_lower_2, red_upper_2)
        red_mask = clean_mask(cv2.bitwise_or(red_mask_1, red_mask_2))

        yellow_mask = clean_mask(cv2.inRange(hsv, yellow_lower, yellow_upper))
        green_mask = clean_mask(cv2.inRange(hsv, green_lower, green_upper))

        red_raw_area = cv2.countNonZero(red_mask)
        yellow_raw_area = cv2.countNonZero(yellow_mask)
        green_raw_area = cv2.countNonZero(green_mask)

        red_circle_area = get_circular_area(red_mask)
        yellow_circle_area = get_circular_area(yellow_mask)
        green_circle_area = get_circular_area(green_mask)

        # Pick the dominant color above min_area. Yellow -> SLOW (prepare to stop).
        # Winner must beat runner-up by WIN_MARGIN — prevents red/yellow flip-flop
        # when both lenses are partially visible.
        areas = {
            "STOP": red_circle_area,
            "SLOW": yellow_circle_area,
            "GO": green_circle_area,
        }
        sorted_areas = sorted(areas.items(), key=lambda kv: kv[1], reverse=True)
        (winner, winner_area), (_runner, runner_area) = sorted_areas[0], sorted_areas[1]

        if winner_area <= min_area:
            raw_signal = "UNKNOWN"
        elif runner_area > 0 and winner_area < runner_area * WIN_MARGIN:
            raw_signal = "UNKNOWN"  # too close to call
        else:
            raw_signal = winner

        signal = smooth_signal(raw_signal)

        print(
            f"[SIGNAL] {signal:<7} (raw={raw_signal:<7}) "
            f"red={red_circle_area:>5} yellow={yellow_circle_area:>5} green={green_circle_area:>5} "
            f"| raw_r={red_raw_area:>5} raw_y={yellow_raw_area:>5} raw_g={green_raw_area:>5}"
        )

        time.sleep(0.2)

except KeyboardInterrupt:
    print("\nCleaning up...")
    picam2.stop()
