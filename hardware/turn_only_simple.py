#!/usr/bin/env python3
"""
turn_only_simple.py

Minimal Raspberry Pi script: turn in place to face detected person using a
single proportional mapping (no integral or derivative). Simple and easy to
understand — like controlling a servo, but for tank motors.

Usage:
  python3 hardware/turn_only_simple.py --openmv /dev/ttyACM0 --arduino /dev/ttyUSB0

Controls:
  --kp: proportional gain (default 0.6)
  --turn-scale: scale from kp*angle to motor magnitude
  --max-speed: clamp motor commands
  --deadzone: degrees to ignore around center
  --autocalibrate: quick motor sign autodetect using camera feedback
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


# Defaults
DEFAULT_OPENMV = "/dev/ttyACM0"
DEFAULT_ARDUINO = "/dev/ttyUSB0"
BAUD = 115200
IMG_WIDTH = 240
CAM_FOV_DEG = 60.0

# Tunable constants (defined in-script, not via CLI)
KP = 0.9
TURN_SCALE = 2.0
MAX_SPEED = 180
DEADZONE = 3.0
AUTOCALIBRATE = False
INVERT_LEFT = False
INVERT_RIGHT = False


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
            line = ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception as e:
            print('OpenMV read error:', e)
            time.sleep(0.5)
            continue
        if not line:
            continue
        parts = line.split()
        if parts and parts[0] == 'OBJ' and len(parts) >= 4:
            try:
                x = int(parts[1])
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
            print('Write error:', e)


def measure_rotation_response(state, arduino_ser, left_cmd, right_cmd, duration=0.35):
    # Wait for a valid cam_x baseline
    start = time.time()
    baseline = None
    while time.time() - start < 3.0:
        bx = state.get_cam_x()
        if bx is not None:
            baseline = bx
            break
        time.sleep(0.05)
    if baseline is None:
        return 0.0

    end_t = time.time() + duration
    while time.time() < end_t:
        try:
            send_cmd(arduino_ser, left_cmd, right_cmd)
        except Exception:
            pass
        time.sleep(0.05)

    try:
        arduino_ser.write(b"STOP\n")
    except Exception:
        pass
    time.sleep(0.12)
    after = state.get_cam_x()
    if after is None:
        return 0.0
    return abs(after - baseline)


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

    active_sign = 1
    if AUTOCALIBRATE:
        print('Autocalibrating motor sign...')
        score_a = measure_rotation_response(state, arduino_ser, 120, -120)
        score_b = measure_rotation_response(state, arduino_ser, -120, 120)
        print(f'Auto scores: A={score_a:.1f} B={score_b:.1f}')
        if score_b > score_a:
            active_sign = -1
            print('Using inverted pattern')

    print('Starting simple proportional turn controller')
    kp = KP
    turn_scale = TURN_SCALE
    max_speed = MAX_SPEED
    deadzone = DEADZONE
    invert_left = INVERT_LEFT
    invert_right = INVERT_RIGHT

    try:
        prev_l = 0
        prev_r = 0
        while True:
            cx = state.get_cam_x()
            angle = angle_from_cam_x(cx)
            if angle is None:
                l = 0
                r = 0
            else:
                if abs(angle) <= deadzone:
                    err = 0.0
                else:
                    err = -angle
                turn_output = kp * err
                diff = int(max(-max_speed, min(max_speed, turn_output * turn_scale)))
                l = active_sign * diff
                r = -active_sign * diff
                if invert_left:
                    l = -l
                if invert_right:
                    r = -r

            send_cmd(arduino_ser, l, r)
            prev_l = l
            prev_r = r
            print(f"CAM_X={cx} ANGLE={angle} L={l} R={r}")
            time.sleep(0.03)

    except KeyboardInterrupt:
        print('Stopping: sending STOP')
        try:
            arduino_ser.write(b"STOP\n")
        except Exception:
            pass
    finally:
        openmv_ser.close()
        arduino_ser.close()


if __name__ == '__main__':
    main()
