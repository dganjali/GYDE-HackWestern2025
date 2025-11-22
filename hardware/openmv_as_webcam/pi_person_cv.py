#!/usr/bin/env python3
"""
pi_person_cv.py

Run on the Raspberry Pi. Reads length-prefixed JPEG frames from the OpenMV
serial (USB CDC) device and runs a simple OpenCV person detector (HOG).

Outputs detection lines compatible with the existing pipeline:
  OBJ <cx> <cy> <dist_m>\n
Also optionally forwards the OBJ line to an Arduino serial port.

Constants are defined in-script. Edit if your devices use different ports.
"""

import time
import sys
import cv2
import numpy as np
import serial

# In-script constants (edit these)
OPENMV_PORT = '/dev/ttyACM0'
OPENMV_BAUD = 115200
ARDUINO_PORT = '/dev/ttyUSB0'   # set to None to disable forwarding
ARDUINO_BAUD = 115200
IMG_W = 320
IMG_H = 240
DISPLAY = False    # set True to show OpenCV window on Pi (needs X)

# Distance estimation params
KNOWN_WIDTH_M = 0.35
FOCAL_PIX = 300.0


def read_exact(ser, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def main():
    try:
        ser = serial.Serial(OPENMV_PORT, OPENMV_BAUD, timeout=1)
    except Exception as e:
        print('Failed to open OpenMV port:', e)
        sys.exit(1)

    arduino = None
    if ARDUINO_PORT:
        try:
            arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
        except Exception as e:
            print('Warning: failed to open Arduino port:', e)
            arduino = None

    # HOG person detector
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    print('Listening for frames...')
    frames = 0
    t0 = time.time()

    while True:
        # read 4-byte length header
        hdr = read_exact(ser, 4)
        if not hdr:
            # timeout or disconnect
            continue
        length = int.from_bytes(hdr, 'little')
        if length <= 0 or length > 200000:
            # invalid length, skip
            continue
        data = read_exact(ser, length)
        if data is None:
            continue

        # decode JPEG
        arr = np.frombuffer(data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            continue
        frames += 1

        # run people detector (scale for speed)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        boxes, weights = hog.detectMultiScale(gray, winStride=(8, 8), padding=(16, 16), scale=1.05)

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
            out = f"OBJ {cx} {cy} {dist_m:.3f}\n"
        else:
            out = "OBJ -1 -1 -1\n"

        # print and optionally forward
        print(out.strip())
        if arduino:
            try:
                arduino.write(out.encode('utf-8'))
            except Exception:
                pass

        if DISPLAY:
            if best is not None:
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.imshow('openmv_stream', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # show FPS occasionally
        if frames % 30 == 0:
            now = time.time()
            fps = frames / (now - t0 + 1e-6)
            print(f"Processed frames: {frames} fps~{fps:.1f}")


if __name__ == '__main__':
    main()
