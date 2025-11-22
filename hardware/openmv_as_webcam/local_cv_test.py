#!/usr/bin/env python3
"""
local_cv_test.py

Run the same CV pipeline as the Pi on your laptop for faster iteration.

Features:
- Use your laptop webcam or a video/file/directory of images as the source
- Runs OpenCV HOG person detector (same as Pi consumer)
- Prints OBJ lines: "OBJ <cx> <cy> <dist_m>"
- Optionally computes and prints tank motor commands using the same P-only
  mapping used on the Pi for quick tuning of KP / TURN_SCALE.

Examples:
  # run webcam (index 0), show display and motor commands
  python3 hardware/openmv_as_webcam/local_cv_test.py --display --send-cmds

  # run on a video file
  python3 hardware/openmv_as_webcam/local_cv_test.py --source video.mp4 --display

  # run on a folder of images
  python3 hardware/openmv_as_webcam/local_cv_test.py --source ./frames --display

Edit KP/TURN_SCALE defaults via CLI to match the Pi values when testing.
"""

import argparse
import time
import os
import glob
import sys
import cv2
import numpy as np


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--source', default='0', help='0=webcam index 0, or path to video file or folder of images')
    p.add_argument('--display', action='store_true', help='Show display window')
    p.add_argument('--send-cmds', action='store_true', help='Print tank motor commands based on detection')
    p.add_argument('--kp', type=float, default=1.0, help='Proportional gain')
    p.add_argument('--turn-scale', type=float, default=3.5, help='Turn scale')
    p.add_argument('--max-speed', type=int, default=255, help='Max motor PWM')
    p.add_argument('--deadzone', type=float, default=2.0, help='Degrees deadzone')
    p.add_argument('--img-w', type=int, default=320)
    p.add_argument('--img-h', type=int, default=240)
    p.add_argument('--cam-fov', type=float, default=60.0)
    return p.parse_args()


def clamp(v, lo=-255, hi=255):
    try:
        iv = int(v)
    except Exception:
        return 0
    if iv < lo:
        return lo
    if iv > hi:
        return hi
    return iv


def make_source(src):
    # webcam index
    if src.isdigit():
        return cv2.VideoCapture(int(src)), None
    # directory of images
    if os.path.isdir(src):
        imgs = sorted(glob.glob(os.path.join(src, '*')))
        return None, imgs
    # video file
    if os.path.isfile(src):
        return cv2.VideoCapture(src), None
    raise RuntimeError('Unable to open source: ' + src)


def main():
    args = parse_args()

    cap, img_list = make_source(args.source)

    # HOG detector
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    frame_i = 0
    t0 = time.time()

    try:
        while True:
            if cap is not None:
                ret, frame = cap.read()
                if not ret:
                    break
            else:
                if frame_i >= len(img_list):
                    break
                frame = cv2.imread(img_list[frame_i])
                frame_i += 1

            if frame is None:
                continue

            # resize to target
            frame_resized = cv2.resize(frame, (args.img_w, args.img_h))

            gray = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)
            boxes, weights = hog.detectMultiScale(gray, winStride=(8, 8), padding=(8, 8), scale=1.05)

            best = None
            best_area = 0
            for (x, y, w, h), score in zip(boxes, weights):
                area = w * h
                if area > best_area:
                    best_area = area
                    best = (x, y, w, h)

            if best is not None:
                x, y, w, h = best
                cx = int(x + w / 2)
                cy = int(y + h / 2)
                dist_m = -1.0
                if w > 0:
                    # use same approximate formula as Pi
                    KNOWN_WIDTH_M = 0.35
                    FOCAL_PIX = 300.0
                    dist_m = (KNOWN_WIDTH_M * FOCAL_PIX) / float(w)
                print(f"OBJ {cx} {cy} {dist_m:.3f}")
            else:
                print("OBJ -1 -1 -1")

            # optionally compute motor commands
            if args.send_cmds:
                if best is None:
                    print('T0,0')
                else:
                    offset_pixels = cx - (args.img_w / 2.0)
                    angle = (offset_pixels / (args.img_w / 2.0)) * (args.cam_fov / 2.0)
                    if abs(angle) <= args.deadzone:
                        left = 0
                        right = 0
                    else:
                        err = -angle
                        turn_output = args.kp * err
                        diff = int(turn_output * args.turn_scale)
                        diff = clamp(diff, -args.max_speed, args.max_speed)
                        left = diff
                        right = -diff
                    print(f"T{left:+d},{right:+d}")

            if args.display:
                vis = frame_resized.copy()
                if best is not None:
                    cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(vis, f"{int(dist_m)}m", (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                cv2.imshow('local_cv_test', vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # small throttle
            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        if cap is not None:
            cap.release()
        if args.display:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
