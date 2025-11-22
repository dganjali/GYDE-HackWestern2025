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
import threading
import queue
import os
import glob
import sys
import cv2
import numpy as np


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--source', default='0', help='0=webcam index 0, or path to video file or folder of images')
    # Default to showing a window for easier local testing.
    p.add_argument('--display', action='store_true', default=True, help='Show display window (default: True)')
    p.add_argument('--send-cmds', action='store_true', help='Print tank motor commands based on detection')
    p.add_argument('--kp', type=float, default=1.0, help='Proportional gain')
    p.add_argument('--turn-scale', type=float, default=3.5, help='Turn scale')
    p.add_argument('--max-speed', type=int, default=255, help='Max motor PWM')
    p.add_argument('--deadzone', type=float, default=2.0, help='Degrees deadzone')
    p.add_argument('--img-w', type=int, default=320)
    p.add_argument('--img-h', type=int, default=240)
    p.add_argument('--cam-fov', type=float, default=60.0)
    p.add_argument('--hold-time', type=float, default=0.6, help='Seconds to hold the last detection before clearing')
    p.add_argument('--smooth-alpha', type=float, default=0.6, help='Exponential smoothing alpha for bbox (0-1, higher = more responsive)')
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

    print(f"[local_cv_test] starting with source={args.source!r} display={args.display} send_cmds={args.send_cmds}")
    cap, img_list = make_source(args.source)
    if cap is not None:
        try:
            opened = cap.isOpened()
        except Exception:
            opened = False
        print(f"[local_cv_test] VideoCapture created. isOpened={opened}")
        if not opened:
            print("[local_cv_test] Warning: VideoCapture not opened - check webcam index or file path")
    else:
        print(f"[local_cv_test] Image list source with {len(img_list)} items")

    # HOG detector
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    frame_i = 0
    t0 = time.time()

    # Use a background thread for detection to keep the GUI responsive.
    frame_q = queue.Queue(maxsize=1)
    result_q = queue.Queue(maxsize=2)
    stop_event = threading.Event()

    def detector_loop():
        while not stop_event.is_set():
            try:
                frame = frame_q.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                boxes, weights = hog.detectMultiScale(gray, winStride=(8, 8), padding=(8, 8), scale=1.05)

                best = None
                best_area = 0
                for (bx, by, bw, bh), score in zip(boxes, weights):
                    area = bw * bh
                    if area > best_area:
                        best_area = area
                        best = (bx, by, bw, bh)

                dist_m = -1.0
                if best is not None:
                    bx, by, bw, bh = best
                    if bw > 0:
                        KNOWN_WIDTH_M = 0.35
                        FOCAL_PIX = 300.0
                        dist_m = (KNOWN_WIDTH_M * FOCAL_PIX) / float(bw)

                # push result (may block if result_q full)
                try:
                    result_q.put_nowait((best, dist_m))
                except queue.Full:
                    # replace oldest
                    try:
                        _ = result_q.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        result_q.put_nowait((best, dist_m))
                    except queue.Full:
                        pass
            except Exception:
                # any error in detection loop should not kill the thread
                continue

    det_thread = threading.Thread(target=detector_loop, daemon=True)
    det_thread.start()

    try:
        last_print = time.time()
        frames = 0
        # temporal smoothing / persistence
        last_seen = None  # (bx, by, bw, bh, dist)
        last_seen_ts = 0.0
        smoothed = None  # (bx, by, bw, bh, dist)
        hold_time = args.hold_time
        alpha = float(args.smooth_alpha)
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

            # send latest frame to detector (overwrite if queue full)
            try:
                frame_q.put_nowait(frame_resized.copy())
            except queue.Full:
                try:
                    _ = frame_q.get_nowait()
                except queue.Empty:
                    pass
                try:
                    frame_q.put_nowait(frame_resized.copy())
                except queue.Full:
                    pass

            # get latest detection result if available (drain queue to keep latest)
            best = None
            dist_m = -1.0
            try:
                while True:
                    best, dist_m = result_q.get_nowait()
            except queue.Empty:
                pass

            now = time.time()
            if best is not None:
                bx, by, bw, bh = best
                # update persistence
                last_seen = (bx, by, bw, bh, dist_m)
                last_seen_ts = now
                if smoothed is None:
                    smoothed = (float(bx), float(by), float(bw), float(bh), float(dist_m))
                else:
                    sbx, sby, sbw, sbh, sdist = smoothed
                    sbx = alpha * float(bx) + (1.0 - alpha) * sbx
                    sby = alpha * float(by) + (1.0 - alpha) * sby
                    sbw = alpha * float(bw) + (1.0 - alpha) * sbw
                    sbh = alpha * float(bh) + (1.0 - alpha) * sbh
                    sdist = alpha * float(dist_m) + (1.0 - alpha) * sdist
                    smoothed = (sbx, sby, sbw, sbh, sdist)
                # print smoothed center
                scx = int(smoothed[0] + smoothed[2] / 2.0)
                scy = int(smoothed[1] + smoothed[3] / 2.0)
                print(f"OBJ {scx} {scy} {smoothed[4]:.3f}")
            else:
                # no fresh detection: if we have a recent last_seen, hold it
                if last_seen is not None and (now - last_seen_ts) <= hold_time and smoothed is not None:
                    scx = int(smoothed[0] + smoothed[2] / 2.0)
                    scy = int(smoothed[1] + smoothed[3] / 2.0)
                    print(f"OBJ {scx} {scy} {smoothed[4]:.3f}")
                else:
                    # clear
                    last_seen = None
                    smoothed = None
                    print("OBJ -1 -1 -1")

            # optionally compute motor commands (use held/smoothed detection)
            if args.send_cmds:
                if smoothed is None:
                    print('T0,0')
                else:
                    sbx, sby, sbw, sbh, sdist = smoothed
                    scx = int(sbx + sbw / 2.0)
                    offset_pixels = scx - (args.img_w / 2.0)
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
                # draw only the smoothed/held detection box if available
                if smoothed is not None:
                    sbx, sby, sbw, sbh, sdist = smoothed
                    dib_x = int(round(sbx))
                    dib_y = int(round(sby))
                    dib_w = int(round(sbw))
                    dib_h = int(round(sbh))
                    cv2.rectangle(vis, (dib_x, dib_y), (dib_x + dib_w, dib_y + dib_h), (0, 255, 0), 2)
                    cv2.putText(vis, f"{int(sdist)}m", (dib_x, dib_y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                cv2.imshow('local_cv_test', vis)
                # Give the GUI some time to update; 30ms is reasonable for local testing.
                if cv2.waitKey(30) & 0xFF == ord('q'):
                    break

            # small throttle
            frames += 1
            if time.time() - last_print >= 2.0:
                now = time.time()
                fps = frames / (now - last_print)
                print(f"[local_cv_test] fps={fps:.1f}")
                frames = 0
                last_print = now
            time.sleep(0.005)

    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        det_thread.join(timeout=0.5)
        if cap is not None:
            cap.release()
        if args.display:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
