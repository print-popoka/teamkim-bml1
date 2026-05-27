"""YOLOv8n object detection on Raspberry Pi camera (BML1 week 9).

Captures frames from the Pi Camera (CSI), runs YOLOv8n inference on CPU,
saves sampled annotated frames to Desktop, previews via cv2.imshow.

Run: python yolo.py     (quit with 'q')

NOTE: Update `output_dir` to match your team username on the Pi
(e.g., /home/teamkim/Desktop/).
"""

import argparse
import os


def parse_args():
    parser = argparse.ArgumentParser(description="YOLOv8n object detection on Pi camera")
    parser.add_argument("--width", type=int, default=320, help="camera width")
    parser.add_argument("--height", type=int, default=240, help="camera height")
    parser.add_argument("--save-every", type=int, default=30, help="save every N frames")
    parser.add_argument("--no-save", action="store_true", help="disable annotated frame saving")
    parser.add_argument("--output-dir", default="/home/team2/Desktop/")
    return parser.parse_args()


args = parse_args()

import cv2
from picamera2 import Picamera2
from ultralytics import YOLO

picam2 = Picamera2()
picam2.preview_configuration.main.size = (args.width, args.height)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()

model = YOLO("yolov8n.pt")

if not args.no_save:
    os.makedirs(args.output_dir, exist_ok=True)

frame_count = 0
print("[INFO] Starting YOLOv8 detection...")
print(f"[INFO] size={args.width}x{args.height}")
if args.no_save:
    print("[INFO] Annotated frame saving disabled.")
else:
    print(f"[INFO] Saving one annotated frame every {args.save_every} frames.")

try:
    while True:
        frame = picam2.capture_array()

        results = model(frame, verbose=False)
        annotated_frame = results[0].plot()

        if not args.no_save and args.save_every > 0 and frame_count % args.save_every == 0:
            output_path = os.path.join(args.output_dir, f"frame_{frame_count:04d}.jpg")
            cv2.imwrite(output_path, annotated_frame)
            print(f"[INFO] Saved: {output_path}")
        frame_count += 1

        cv2.imshow("YOLOv8 Detection", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    pass
finally:
    cv2.destroyAllWindows()
    picam2.stop()
