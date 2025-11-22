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
# Default PID gains (tweak below or via CLI)
KP = 3.0
KI = 0.0
KD = 8.5
# control loop interval (s)
DT = 0.03  # ~33 Hz for faster response
# scale to convert PID output (degrees) -> motor speed
TURN_SCALE = 3.5
# clamp for motor command magnitude
MAX_SPEED = 220
# integral anti-windup default
INTEGRAL_LIMIT = 100.0
# slew rate default (units per second)
SLEW_RATE = 800.0
# derivative filter time constant (s). Small values = less filtering.
D_FILTER_TAU = 0.04
# Adaptive scaling configuration: make aggressive scaling less extreme to avoid overshoot
ADAPTIVE_DENOM = 30.0
ADAPTIVE_MAX_EXTRA = 1.0  # max additional multiplier over 1.0 (so max scale = 1 + ADAPTIVE_MAX_EXTRA)


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
        self.seq = None
        self.openmv_ts = None
        self.recv_ms = None

    def set_cam_x(self, x):
        with self.lock:
            self.cam_x = x

    def set_meta(self, seq, ts, recv_ms):
        with self.lock:
            self.seq = seq
            self.openmv_ts = ts
            self.recv_ms = recv_ms

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
        if parts and parts[0] == 'OBJ':
            try:
                if len(parts) >= 6:
                    seq = int(parts[1])
                    x = int(parts[2])
                    # y = int(parts[3])
                    # d = float(parts[4])
                    ts = int(parts[5])
                elif len(parts) >= 4:
                    seq = None
                    x = int(parts[1])
                    ts = None
                else:
                    state.set_cam_x(None)
                    continue
            except Exception:
                state.set_cam_x(None)
                continue
            if x >= 0:
                state.set_cam_x(x)
                recv_ms = int(time.time() * 1000)
                state.set_meta(seq, ts, recv_ms)
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
    p.add_argument('--invert-left', action='store_true', help='Invert left motor sign')
    p.add_argument('--invert-right', action='store_true', help='Invert right motor sign')
    p.add_argument('--deadzone', type=float, default=0.0, help='Degrees deadzone around center to ignore small errors')
    p.add_argument('--autocalibrate', action='store_true', help='Auto-detect sign pattern using camera feedback')
    p.add_argument('--kp', type=float, default=KP, help='Proportional gain')
    p.add_argument('--ki', type=float, default=KI, help='Integral gain')
    p.add_argument('--kd', type=float, default=KD, help='Derivative gain')
    p.add_argument('--simple-proportional', action='store_true', help='Use a simple P-only controller (no I/D) similar to a servo')
    p.add_argument('--turn-scale', type=float, default=TURN_SCALE, help='Scale from PID output to motor differential')
    p.add_argument('--dt', type=float, default=DT, help='Control loop interval in seconds')
    p.add_argument('--max-speed', type=int, default=MAX_SPEED, help='Max motor speed magnitude')
    p.add_argument('--int-limit', type=float, default=INTEGRAL_LIMIT, help='Integral windup clamp')
    p.add_argument('--slew-rate', type=float, default=SLEW_RATE, help='Max change in motor command per second')
    p.add_argument('--d-filter-tau', type=float, default=D_FILTER_TAU, help='Derivative low-pass filter time constant (s)')
    args = p.parse_args()

    invert_left = args.invert_left
    invert_right = args.invert_right
    deadzone = args.deadzone
    autocal = args.autocalibrate
    # PID gains (can override via CLI)
    kp = args.kp
    ki = args.ki
    kd = args.kd
    simple_prop = args.simple_proportional
    turn_scale = args.turn_scale
    dt = args.dt
    max_speed = args.max_speed
    int_limit = args.int_limit
    slew_rate = args.slew_rate
    d_filter_tau = args.d_filter_tau

    # print active tuning parameters
    print(f"PID params: kp={kp} ki={ki} kd={kd} dt={dt} d_tau={d_filter_tau} turn_scale={turn_scale} max_speed={max_speed} slew_rate={slew_rate}")

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

    def measure_rotation_response(test_left, test_right, test_time=0.4, settle=0.1):
        # wait for a valid cam_x
        start = time.time()
        baseline = None
        while time.time() - start < 5.0:
            bx = state.get_cam_x()
            if bx is not None:
                baseline = bx
                break
            time.sleep(0.05)
        if baseline is None:
            return 0.0

        # send test command repeatedly for duration
        end_t = time.time() + test_time
        while time.time() < end_t:
            try:
                send_cmd(arduino_ser, test_left, test_right)
            except Exception:
                pass
            time.sleep(0.05)

        # stop
        try:
            arduino_ser.write(b"STOP\n")
        except Exception:
            pass

        # wait to settle
        time.sleep(settle)
        after = state.get_cam_x()
        if after is None:
            return 0.0
        return abs(after - baseline)

    # active_sign selects which command pattern to use: left = active_sign*diff, right = -active_sign*diff
    active_sign = 1
    if autocal:
        print("Autocalibrating motor sign pattern using camera feedback...")
        diff_test = 120
        score_a = measure_rotation_response(diff_test, -diff_test)
        score_b = measure_rotation_response(-diff_test, diff_test)
        print(f"Autocal scores: patternA={score_a:.2f}, patternB={score_b:.2f}")
        if score_b > score_a:
            active_sign = -1
            print("Using inverted pattern (left=-diff, right=+diff)")
        else:
            active_sign = 1
            print("Using default pattern (left=+diff, right=-diff)")

    integral = 0.0
    prev_err = 0.0
    prev_derivative = 0.0
    prev_left = 0
    prev_right = 0

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
                # apply deadzone to avoid twitching
                if abs(angle) <= deadzone:
                    err = 0.0
                else:
                    err = -angle
                    if simple_prop:
                        # simple proportional control (servo-like): direct mapping
                        turn_output = kp * err
                        eff_turn_scale = turn_scale
                        diff = int(max(-max_speed, min(max_speed, turn_output * eff_turn_scale)))
                    else:
                        integral += err * dt
                        # anti-windup clamp
                        if integral > int_limit:
                            integral = int_limit
                        elif integral < -int_limit:
                            integral = -int_limit

                        # derivative (filtered) to reduce noise-driven D spikes
                        derivative_raw = (err - prev_err) / dt
                        alpha = dt / (d_filter_tau + dt) if d_filter_tau > 0.0 else 1.0
                        derivative = alpha * derivative_raw + (1.0 - alpha) * prev_derivative
                        prev_derivative = derivative
                        prev_err = err

                        turn_output = kp * err + ki * integral + kd * derivative

                        # adaptive turn scaling: be more conservative to avoid overshoot
                        adaptive_extra = min(abs(err) / ADAPTIVE_DENOM, ADAPTIVE_MAX_EXTRA)
                        adaptive_scale = 1.0 + adaptive_extra
                        eff_turn_scale = turn_scale * adaptive_scale

                        diff = int(max(-max_speed, min(max_speed, turn_output * eff_turn_scale)))
                # in-place rotation using active_sign
                left = active_sign * diff
                right = -active_sign * diff

                # allow per-motor inversion if wiring requires it
                if invert_left:
                    left = -left
                if invert_right:
                    right = -right

            # slew rate limiting to avoid sudden large commands
            max_step = slew_rate * dt
            delta_l = left - prev_left
            if delta_l > max_step:
                left = int(prev_left + max_step)
            elif delta_l < -max_step:
                left = int(prev_left - max_step)

            delta_r = right - prev_right
            if delta_r > max_step:
                right = int(prev_right + max_step)
            elif delta_r < -max_step:
                right = int(prev_right - max_step)

            send_cmd(arduino_ser, left, right)
            prev_left = left
            prev_right = right
            # print latency info if available
            with state.lock:
                meta_seq = state.seq
                meta_ts = state.openmv_ts
                meta_recv = state.recv_ms
            if meta_ts is not None and meta_recv is not None:
                lag = meta_recv - meta_ts
                print(f"SEQ={meta_seq} LAG={lag}ms CAM_X={cam_x} ANGLE={angle} L={left} R={right}")
            else:
                print(f"CAM_X={cam_x} ANGLE={angle} L={left} R={right}")
            time.sleep(dt)

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
