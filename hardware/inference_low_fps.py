## THis is for single MP4 upload, no real time streaming (base)

import sys
import time
from pathlib import Path
import cv2
from ultralytics import YOLO

# --- CONFIG ---
MODEL_PATH = 'best.pt'  # Path to trained weights
VIDEO_PATH = 'kid_drowning.mp4'  # Change to your test video file
FPS = 1  # Set to 1 or 2 for low frame rate

# --- LOAD MODEL ---
model = YOLO(MODEL_PATH)

# --- OPEN VIDEO ---
cap = cv2.VideoCapture(VIDEO_PATH)
if not cap.isOpened():
    print(f"Error: Could not open video {VIDEO_PATH}")
    sys.exit(1)

video_fps = cap.get(cv2.CAP_PROP_FPS)
frame_interval = int(round(video_fps / FPS))
frame_idx = 0
print(f"Processing video at {FPS} FPS (every {frame_interval} frames)...")

while True:
    ret, frame = cap.read()
    if not ret:
        break
    if frame_idx % frame_interval == 0:
        # Run inference
        results = model(frame)
        drowning_found = False
        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                if cls == 0:  # Drowning
                    drowning_found = True
                    print(f"Frame {frame_idx}: Drowning: {conf*100:.1f}%")
        if not drowning_found:
            print(f"Frame {frame_idx}: No Drowning detected.")
    frame_idx += 1

cap.release()
print("Done.") 
