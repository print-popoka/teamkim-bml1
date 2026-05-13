"""YOLO + HSV hybrid traffic-light detection (lecture's "advanced" pipeline).

Flow:
  1. YOLOv8n detects 'traffic light' boxes (COCO class 9).
  2. Crop each box.
  3. Run HSV red/green check inside that crop only — false positives from
     red shirts / brake lights / green plants disappear because they are
     never inside a 'traffic light' bounding box.
  4. Decide STOP / GO / UNKNOWN with 5-frame temporal smoothing.

CPU only — sequential YOLO+HSV in one process is fine. The "don't run
YOLO and HSV simultaneously" rule is about running them as two parallel
processes.

Run: python camera/yolo_hsv.py     (quit with 'q' in the preview window)
"""

import time
from collections import Counter, deque

import cv2
import numpy as np
from picamera2 import Picamera2
from ultralytics import YOLO

TRAFFIC_LIGHT_CLASS_ID = 9  # COCO class index for "traffic light"
CONF_THRESHOLD = 0.25

# Camera --------------------------------------------------------------
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"  # numpy order is BGR
picam2.configure("preview")
picam2.start()
time.sleep(1)

model = YOLO("yolov8n.pt")

# HSV ranges ----------------------------------------------------------
red_lower_1 = np.array([0, 50, 50])
red_upper_1 = np.array([10, 255, 255])
red_lower_2 = np.array([165, 50, 50])
red_upper_2 = np.array([179, 255, 255])

green_lower = np.array([40, 60, 60])
green_upper = np.array([90, 255, 255])

MIN_COLOR_RATIO = 0.05  # at least 5% of the crop must match
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

# Temporal smoothing --------------------------------------------------
SMOOTH_WINDOW = 5
SMOOTH_MIN_VOTES = 3
history = deque(maxlen=SMOOTH_WINDOW)


def classify_crop(bgr_crop):
    """Return ('STOP'|'GO'|'UNKNOWN', red_ratio, green_ratio)."""
    if bgr_crop.size == 0:
        return "UNKNOWN", 0.0, 0.0

    hsv = cv2.cvtColor(bgr_crop, cv2.COLOR_BGR2HSV)

    red = cv2.bitwise_or(
        cv2.inRange(hsv, red_lower_1, red_upper_1),
        cv2.inRange(hsv, red_lower_2, red_upper_2),
    )
    green = cv2.inRange(hsv, green_lower, green_upper)

    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, KERNEL)
    green = cv2.morphologyEx(green, cv2.MORPH_OPEN, KERNEL)

    total = bgr_crop.shape[0] * bgr_crop.shape[1]
    red_ratio = cv2.countNonZero(red) / total
    green_ratio = cv2.countNonZero(green) / total

    if red_ratio > MIN_COLOR_RATIO and red_ratio > green_ratio:
        return "STOP", red_ratio, green_ratio
    if green_ratio > MIN_COLOR_RATIO and green_ratio > red_ratio:
        return "GO", red_ratio, green_ratio
    return "UNKNOWN", red_ratio, green_ratio


def smooth(raw):
    history.append(raw)
    if len(history) < SMOOTH_WINDOW:
        return raw
    top, votes = Counter(history).most_common(1)[0]
    return top if votes >= SMOOTH_MIN_VOTES else "UNKNOWN"


print("[INFO] Starting YOLO + HSV hybrid traffic light detection...")

try:
    while True:
        frame = picam2.capture_array()

        results = model(
            frame,
            classes=[TRAFFIC_LIGHT_CLASS_ID],
            conf=CONF_THRESHOLD,
            verbose=False,
        )
        r = results[0]

        best_label = "UNKNOWN"
        best_conf = 0.0
        best_box = None

        if r.boxes is not None and len(r.boxes) > 0:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < best_conf:
                    continue
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                crop = frame[max(0, y1) : y2, max(0, x1) : x2]
                label, red_r, green_r = classify_crop(crop)
                if label != "UNKNOWN":
                    best_label = label
                    best_conf = conf
                    best_box = (x1, y1, x2, y2, red_r, green_r)

        signal = smooth(best_label)

        if best_box is not None:
            x1, y1, x2, y2, red_r, green_r = best_box
            print(
                f"[SIGNAL] {signal:<7} (raw={best_label:<7}) "
                f"box=({x1},{y1},{x2},{y2}) conf={best_conf:.2f} "
                f"red={red_r:.2%} green={green_r:.2%}"
            )
            color = (
                (0, 0, 255) if best_label == "STOP"
                else (0, 255, 0) if best_label == "GO"
                else (200, 200, 200)
            )
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                frame, signal, (x1, max(0, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2,
            )
        else:
            print(f"[SIGNAL] {signal:<7} (no traffic light detected)")

        cv2.imshow("YOLO+HSV Traffic Light", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    pass
finally:
    cv2.destroyAllWindows()
    picam2.stop()
