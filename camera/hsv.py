"""HSV traffic-light color detection (BML1 Camera/OpenCV lecture).

Pipeline: Camera frame -> HSV -> red/green masks -> pixel-area count
         -> STOP / GO / UNKNOWN signal.

Run: python hsv.py     (quit with Ctrl+C)
"""

import argparse
import time


def parse_args():
    parser = argparse.ArgumentParser(description="Basic HSV traffic-light color detection")
    parser.add_argument("--width", type=int, default=320, help="camera width")
    parser.add_argument("--height", type=int, default=240, help="camera height")
    parser.add_argument("--print-every", type=int, default=5, help="print every N frames")
    parser.add_argument("--sleep", type=float, default=0.05, help="loop sleep in seconds")
    return parser.parse_args()


args = parse_args()

import cv2
import numpy as np
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.preview_configuration.main.size = (args.width, args.height)
picam2.preview_configuration.main.format = "RGB888"  # libcamera quirk: numpy order is BGR
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

area_scale = (args.width * args.height) / (640 * 480)
min_area = max(125, int(500 * area_scale))

print("[INFO] Starting HSV traffic light detection...")
print(f"[INFO] size={args.width}x{args.height} print_every={args.print_every} sleep={args.sleep}s")

try:
    frame_id = 0
    while True:
        frame = picam2.capture_array()
        frame_id += 1

        # picamera2 RGB888 -> BGR in numpy, so BGR2HSV.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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

        if args.print_every > 0 and frame_id % args.print_every == 0:
            print(f"[SIGNAL] {signal} | red_area={red_area}, green_area={green_area}")

        if args.sleep > 0:
            time.sleep(args.sleep)

except KeyboardInterrupt:
    print("\nCleaning up...")
finally:
    picam2.stop()
