"""HSV + circularity + temporal smoothing for traffic-light detection.

Improvements over the lecture baseline:
  - picamera2's "RGB888" actually returns BGR-ordered ndarrays, so we use
    COLOR_BGR2HSV (not RGB2HSV) — fixes "red looks like blue" bugs.
  - ROI: only the upper portion of the frame is searched (signals are overhead).
  - Morphological open+close to kill speckle and fill small gaps.
  - Wider red range and lower S floor — real LED reds desaturate at distance.
  - Temporal smoothing: 5-frame majority vote, avoids flickering decisions.
  - Throttles debug printing so terminal I/O does not dominate the loop.

Run: python camera/hsv_circle.py     (quit with Ctrl+C)
"""

import argparse
import time
from collections import Counter, deque

def parse_args():
    parser = argparse.ArgumentParser(description="Fast HSV circle traffic-light detection")
    parser.add_argument("--width", type=int, default=320, help="camera width")
    parser.add_argument("--height", type=int, default=240, help="camera height")
    parser.add_argument(
        "--roi-ratio",
        type=float,
        default=0.6,
        help="fraction of image height to scan from the top",
    )
    parser.add_argument("--print-every", type=int, default=5, help="print every N frames")
    parser.add_argument("--sleep", type=float, default=0.05, help="loop sleep in seconds")
    return parser.parse_args()


args = parse_args()

import cv2
import numpy as np
from picamera2 import Picamera2

# Single source of truth for HSV color thresholds + circularity + smoothing.
# Imported with snake_case aliases so the downstream loop below keeps using
# its existing names. Area/radius bounds stay local because this CLI supports
# arbitrary --width/--height (production detector uses fixed resolution).
from perception.traffic_light import (
    RED_LOWER_1 as red_lower_1,
    RED_UPPER_1 as red_upper_1,
    RED_LOWER_2 as red_lower_2,
    RED_UPPER_2 as red_upper_2,
    GREEN_LOWER as green_lower,
    GREEN_UPPER as green_upper,
    MIN_CIRCULARITY as min_circularity,
    WIN_MARGIN,
    SMOOTH_WINDOW,
    SMOOTH_MIN_VOTES,
)

# Camera --------------------------------------------------------------
picam2 = Picamera2()
picam2.preview_configuration.main.size = (args.width, args.height)
picam2.preview_configuration.main.format = "RGB888"  # libcamera quirk: numpy order is BGR
picam2.configure("preview")
picam2.start()
time.sleep(1)

# HSV color thresholds + circularity + WIN_MARGIN + SMOOTH_* are imported
# above from perception/traffic_light.py — calibration history lives there.
# Tuning notes (2026-05-17, printed-paper alpha test, 18 hsv_picker samples):
#   RED   : S>=150, V>=100  (RED_ON S 211-255 V 173-198 ; RED_OFF S<=143, V<=75)
#   GREEN : S>=135           (GREEN_ON S 140-171 ; GREEN_OFF S<=128)
#   YELLOW_OFF (H 35-37) safely below green_lower H=40.
# Real test will use MORE saturated colors -> thresholds are conservative-safe.

# Shape thresholds — resolution-scaled (debug CLI runs at arbitrary size,
# production detector uses fixed picamera2 resolution and the fixed values
# baked into perception/traffic_light.py).
area_scale = (args.width * args.height) / (640 * 480)
length_scale = min(args.width / 640, args.height / 480)
min_area = max(50, int(200 * area_scale))
min_radius = max(3, int(6 * length_scale))
max_radius = max(30, int(140 * length_scale))

# Temporal smoothing buffer (window size from perception import above).
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
print(
    f"[INFO] size={args.width}x{args.height} roi={args.roi_ratio:.2f} "
    f"print_every={args.print_every} sleep={args.sleep}s"
)
print("[INFO] Tune red/green HSV ranges if [DEBUG] areas look wrong under your lighting.")

try:
    frame_id = 0
    while True:
        frame = picam2.capture_array()
        frame_id += 1

        # ROI: upper portion of the image (traffic lights are overhead).
        h, w = frame.shape[:2]
        roi_h = max(1, min(h, int(h * args.roi_ratio)))
        roi = frame[:roi_h, :]

        # picamera2 RGB888 -> BGR in numpy, so BGR2HSV.
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        red_mask_1 = cv2.inRange(hsv, red_lower_1, red_upper_1)
        red_mask_2 = cv2.inRange(hsv, red_lower_2, red_upper_2)
        red_mask = clean_mask(cv2.bitwise_or(red_mask_1, red_mask_2))

        green_mask = clean_mask(cv2.inRange(hsv, green_lower, green_upper))

        red_raw_area = cv2.countNonZero(red_mask)
        green_raw_area = cv2.countNonZero(green_mask)

        red_circle_area = get_circular_area(red_mask)
        green_circle_area = get_circular_area(green_mask)

        # RED -> STOP, GREEN -> GO. Winner must beat the other by WIN_MARGIN.
        if red_circle_area <= min_area and green_circle_area <= min_area:
            raw_signal = "UNKNOWN"
        elif red_circle_area > green_circle_area:
            raw_signal = "STOP" if red_circle_area >= green_circle_area * WIN_MARGIN else "UNKNOWN"
        else:
            raw_signal = "GO" if green_circle_area >= red_circle_area * WIN_MARGIN else "UNKNOWN"

        signal = smooth_signal(raw_signal)

        if args.print_every > 0 and frame_id % args.print_every == 0:
            print(
                f"[SIGNAL] {signal:<7} (raw={raw_signal:<7}) "
                f"red_circle={red_circle_area:>5} green_circle={green_circle_area:>5} "
                f"| red_raw={red_raw_area:>5} green_raw={green_raw_area:>5}"
            )

        if args.sleep > 0:
            time.sleep(args.sleep)

except KeyboardInterrupt:
    print("\nCleaning up...")
finally:
    picam2.stop()
