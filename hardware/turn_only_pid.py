#!/usr/bin/env python3
"""
turn_only_pid.py

Minimal Raspberry Pi test script that only turns the robot to face the detected person.
It reads OBJ x y d lines from the OpenMV serial, runs a PID on heading (camera x -> angle),
and sends in-place rotation commands to the Arduino in the format "T<left>,<right>\n".

Usage:
  python3 hardware/turn_only_pid.py --openmv /dev/ttyACM1 --arduino /dev/ttyUSB1

This intentionally ignores distance and only adjusts heading.
"""

import argparse
import threading
import time
import sys

try:
    import serial
except Exception:
    print("pyserial required: pip3 install pyserial")
    raise


# ----- Defaults (match repo settings) -----
DEFAULT_OPENMV = "/dev/ttyACM0"
DEFAULT_ARDUINO = "/dev/ttyUSB0"
BAUD = 115200

# Camera and PID config
IMG_WIDTH = 240
CAM_FOV_DEG = 60.0
KP = 0.8
KI = 0.0
KD = 0.02
DT = 0.1
TURN_SCALE = 3.0
MAX_SPEED = 200


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


class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.cam_x = None

    def set_cam_x(self, x):
        with self.lock:
            self.cam_x = x

    def get_cam_x(self):
        with self.lock:
            return self.cam_x


def openmv_reader(ser, state):
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
        except Exception as e:
            print("OpenMV read error:", e)
            time.sleep(0.5)
            continue
        if not line:
            continue
        parts = line.split()
        if parts and parts[0] == 'OBJ' and len(parts) >= 4:
            try:
                x = int(parts[1])
                # y = int(parts[2])
                # d = float(parts[3])
            except Exception:
                state.set_cam_x(None)
                continue
            if x >= 0:
                state.set_cam_x(x)
            else:
                state.set_cam_x(None)


def angle_from_cam_x(cx):
    if cx is None:
        return None
    offset_pixels = cx - (IMG_WIDTH / 2.0)
    angle = (offset_pixels / (IMG_WIDTH / 2.0)) * (CAM_FOV_DEG / 2.0)
    return angle


def send_cmd(ser, left, right):
    left = clamp(left)
    right = clamp(right)
    cmd = "T{:+d},{:+d}\n".format(int(left), int(right))
    try:
        ser.write(cmd.encode('utf-8'))
    except Exception:
        try:
            ser.write(cmd)
        except Exception as e:
            print("Write error:", e)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--openmv', default=DEFAULT_OPENMV)
    p.add_argument('--arduino', default=DEFAULT_ARDUINO)
    p.add_argument('--baud', default=BAUD, type=int)
    args = p.parse_args()

    try:
        openmv_ser = serial.Serial(args.openmv, args.baud, timeout=1)
    except Exception as e:
        print(f"Failed to open OpenMV port {args.openmv}: {e}")
        sys.exit(1)

    try:
        arduino_ser = serial.Serial(args.arduino, args.baud, timeout=1)
    except Exception as e:
        print(f"Failed to open Arduino port {args.arduino}: {e}")
        openmv_ser.close()
        sys.exit(1)

    state = SharedState()
    t = threading.Thread(target=openmv_reader, args=(openmv_ser, state), daemon=True)
    t.start()

    integral = 0.0
    prev_err = 0.0

    print("Starting turn-only PID. Press Ctrl-C to stop.")
    try:
        while True:
            cam_x = state.get_cam_x()
            angle = angle_from_cam_x(cam_x)
            if angle is None:
                # no visual: stop turning
                left = 0
                right = 0
            else:
                err = -angle
                integral += err * DT
                derivative = (err - prev_err) / DT
                turn_output = KP * err + KI * integral + KD * derivative
                prev_err = err

                diff = int(max(-MAX_SPEED, min(MAX_SPEED, turn_output * TURN_SCALE)))
                # in-place rotation
                left = -diff
                right = diff

            send_cmd(arduino_ser, left, right)
            print(f"CAM_X={cam_x} ANGLE={angle} L={left} R={right}")
            time.sleep(DT)

    except KeyboardInterrupt:
        print("Stopping: sending STOP")
        try:
            arduino_ser.write(b"STOP\n")
        except Exception:
            pass
    finally:
        openmv_ser.close()
        arduino_ser.close()


if __name__ == '__main__':
    main()
