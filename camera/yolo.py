"""YOLOv8n object detection on Raspberry Pi camera (BML1 week 9).

Captures frames from the Pi Camera (CSI), runs YOLOv8n inference on CPU,
saves annotated frames to Desktop, previews via cv2.imshow.

Run: python yolo.py     (quit with 'q')

NOTE: Update `output_dir` to match your team username on the Pi
(e.g., /home/teamkim/Desktop/).
"""

import os

import cv2
from picamera2 import Picamera2
from ultralytics import YOLO

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()

model = YOLO("yolov8n.pt")

output_dir = "/home/team2/Desktop/"
os.makedirs(output_dir, exist_ok=True)

frame_count = 0
print("[INFO] Starting YOLOv8 detection...")

while True:
    frame = picam2.capture_array()

    results = model(frame)
    annotated_frame = results[0].plot()

    output_path = os.path.join(output_dir, f"frame_{frame_count:04d}.jpg")
    cv2.imwrite(output_path, annotated_frame)
    print(f"[INFO] Saved: {output_path}")
    frame_count += 1

    cv2.imshow("YOLOv8 Detection", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
