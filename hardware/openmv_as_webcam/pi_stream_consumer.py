#!/usr/bin/env python3
"""
pi_stream_consumer.py

Minimal Raspberry Pi consumer for the OpenMV stream produced by
`openmv_stream.py` (4-byte little-endian length prefix + JPEG bytes).

This script reads frames, decodes JPEG, runs a simple HOG person detector and
emits OBJ messages compatible with the rest of the repo:

  OBJ <cx> <cy> <dist_m>\n

It can also forward those OBJ lines to an Arduino serial port.
Adjust the in-script constants below (ports, focal length, known width).
"""

import time
import sys
import cv2
import numpy as np
import serial

# --- In-script constants ---
OPENMV_PORT = '/dev/ttyACM0'
OPENMV_BAUD = 115200

# If you want the Pi to forward OBJ lines to Arduino, set this to your port.
ARDUINO_PORT = '/dev/ttyUSB0'  # set to None to disable forwarding
ARDUINO_BAUD = 115200

# Simple heading control constants (P-only controller mapping angle->motor pwm)
KP = 1.0
TURN_SCALE = 3.5
MAX_SPEED = 255
DEADZONE = 2.0  # degrees
CAM_FOV_DEG = 60.0
IMG_W = 320

# Image and detection params
DISPLAY = False      # True to show a preview window (requires X)
IMG_W = 320
IMG_H = 240

# Distance estimation params (tune for your camera/lens)
KNOWN_WIDTH_M = 0.35
FOCAL_PIX = 300.0

# Detector tuning
HOG_WINSTRIDE = (8, 8)
HOG_PADDING = (8, 8)
HOG_SCALE = 1.05


def read_exact(ser, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


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


def main():
    try:
        openmv = serial.Serial(OPENMV_PORT, OPENMV_BAUD, timeout=1)
    except Exception as e:
        print('Failed to open OpenMV port', OPENMV_PORT, e, file=sys.stderr)
        sys.exit(1)

    arduino = None
    if ARDUINO_PORT:
        try:
            arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
        except Exception as e:
            print('Warning: failed to open Arduino port', ARDUINO_PORT, e, file=sys.stderr)
            arduino = None

    # HOG person detector (lightweight)
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    frames = 0
    t0 = time.time()

    print('Ready: reading length-prefixed JPEG frames from', OPENMV_PORT)

    try:
        while True:
            hdr = read_exact(openmv, 4)
            if hdr is None:
                # timeout or disconnect; just continue
                continue
            length = int.from_bytes(hdr, 'little')
            if length <= 0 or length > 500000:
                # invalid length; skip
                continue
            data = read_exact(openmv, length)
            if data is None:
                continue

            # decode JPEG
            arr = np.frombuffer(data, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                continue
            frames += 1

            # detection on grayscale for speed
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            boxes, weights = hog.detectMultiScale(gray, winStride=HOG_WINSTRIDE, padding=HOG_PADDING, scale=HOG_SCALE)

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
                    dist_m = (KNOWN_WIDTH_M * FOCAL_PIX) / float(w)
                msg = f"OBJ {cx} {cy} {dist_m:.3f}\n"
            else:
                msg = "OBJ -1 -1 -1\n"

            # print OBJ for debugging
            print(msg.strip())
            sys.stdout.flush()

            # If we have an Arduino, compute simple P-only heading control and send tank command
            if arduino:
                try:
                    if best is None:
                        # stop motors if no detection
                        arduino.write(b"T0,0\n")
                    else:
                        # compute angle from camera x
                        offset_pixels = cx - (IMG_W / 2.0)
                        angle = (offset_pixels / (IMG_W / 2.0)) * (CAM_FOV_DEG / 2.0)
                        if abs(angle) <= DEADZONE:
                            left = 0
                            right = 0
                        else:
                            err = -angle
                            turn_output = KP * err
                            diff = int(turn_output * TURN_SCALE)
                            diff = clamp(diff, -MAX_SPEED, MAX_SPEED)
                            left = diff
                            right = -diff
                        cmd = f"T{left:+d},{right:+d}\n"
                        arduino.write(cmd.encode('utf-8'))
                except Exception:
                    pass

            if DISPLAY:
                vis = img.copy()
                if best is not None:
                    cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.imshow('openmv', vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # occasional FPS print
            if frames % 60 == 0:
                now = time.time()
                fps = frames / (now - t0 + 1e-9)
                print(f'Processed frames: {frames} fps~{fps:.1f}', file=sys.stderr)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            openmv.close()
        except:
            pass
        if arduino:
            try:
                arduino.close()
            except:
                pass


if __name__ == '__main__':
    main()
