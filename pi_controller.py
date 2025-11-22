#!/usr/bin/env python3
"""
Minimal Raspberry Pi controller for the MVP pipeline.

Reads lightweight lines from the OpenMV camera over serial (any human-readable line
that contains a dx value — this script is robust to a few formats), computes
simple linear and angular commands, and forwards them to an Arduino Nano over
serial as lightweight numeric messages:

  L:<linear_m_s>;A:<angular_rad_s>\n

The script is intentionally conservative and configurable via constants at the
top. It attempts to be plug-and-play with the project's existing OpenMV
scripts which print lines containing "dx=" or "dx:" and bounding-box info.
"""

import argparse
import re
import serial
import time
import sys
import threading
from collections import deque

# --- Config ---
DEFAULT_OPENMV_PORT = '/dev/ttyACM0'
DEFAULT_ARDUINO_PORT = '/dev/ttyUSB0'
OPENMV_BAUD = 115200
ARDUINO_BAUD = 115200

IMAGE_WIDTH = 240  # pixels (used to normalize dx)
MAX_LINEAR = 0.25   # m/s, conservative default
MAX_ANGULAR = 1.0   # rad/s

# Controller gains (PID for angular)
PID_KP = 1.6
PID_KI = 0.01
PID_KD = 0.15
KP_LIN = 0.6

# Desired distance (target following distance)
DESIRED_DISTANCE = 0.5  # meters (50 cm target)

CAM_TIMEOUT = 0.25  # seconds without camera input -> send zero vel
# Telemetry thresholds
FRONT_OBSTACLE_CM = 25  # stop if object closer than this

# Control rates
TURN_RATE_HZ = 30.0
MOVE_RATE_HZ = 20.0
SENDER_RATE_HZ = 20.0

# Centering threshold in pixels
CENTER_THRESHOLD_PX = 10


def parse_dx_and_height(line: str):
    """Try to extract dx (horizontal offset in pixels) and a height/box metric.
    Returns (dx:int|None, h:int|None). The function is permissive and looks for
    several patterns commonly found in the repo's OpenMV output.
    """
    # dx patterns: "dx=123", "dx:123", "dx 123", or "dx\t123"
    m = re.search(r'dx[:=]?\s*(-?\d+)', line, flags=re.I)
    dx = int(m.group(1)) if m else None

    # bounding box height (h or height) or y/h pairs, try common signatures
    m2 = re.search(r'h[:=]?\s*(\d+)', line, flags=re.I)
    h = int(m2.group(1)) if m2 else None

    # Some prints include "y <num>\t..." and "x <num>\t y <num>\t"
    # If a bbox height isn't present, we can't reliably estimate distance.
    return dx, h


def estimate_distance_from_height(h: int):
    """Simple inverse relation with a tuned constant. This requires calibration
    for a real robot. We pick a placeholder constant so that larger h -> smaller
    distance.
    """
    if h is None or h <= 0:
        return None
    # simple calibrated constant (needs tuning per camera)
    K = 60.0
    return K / float(h)


def compute_cmds(dx, distance):
    # Normalize dx to [-1,1] where 1 means far right edge
    if dx is None:
        norm_dx = 0.0
    else:
        norm_dx = float(dx) / (IMAGE_WIDTH / 2.0)
        norm_dx = max(-1.0, min(1.0, norm_dx))

    # Angular: drive proportional to normalized dx (simple fallback if PID not used)
    ang = -PID_KP * norm_dx * MAX_ANGULAR

    # Linear: encourage forward motion when target is centered. If distance is
    # known, use it to approach DESIRED_DISTANCE, otherwise reduce when target
    # is off-center.
    if distance is not None:
        # simple P controller toward desired distance
        err = (distance - DESIRED_DISTANCE) / max(DESIRED_DISTANCE, 0.001)
        lin = KP_LIN * err * MAX_LINEAR
        lin = max(-MAX_LINEAR, min(MAX_LINEAR, lin))
    else:
        # fallback: reduce forward speed as target deviates from center
        lin = MAX_LINEAR * max(0.0, 1.0 - abs(norm_dx))

    return lin, ang


def main():
    parser = argparse.ArgumentParser(description='Pi controller - forward OpenMV -> Arduino with simple control')
    parser.add_argument('--openmv-port', default=DEFAULT_OPENMV_PORT)
    parser.add_argument('--openmv-baud', default=OPENMV_BAUD, type=int)
    parser.add_argument('--arduino-port', default=DEFAULT_ARDUINO_PORT)
    parser.add_argument('--arduino-baud', default=ARDUINO_BAUD, type=int)
    parser.add_argument('--dry-run', action='store_true', help='Do not open Arduino serial, print only')
    args = parser.parse_args()

    try:
        cam = serial.Serial(args.openmv_port, args.openmv_baud, timeout=0.1)
        print(f'Opened OpenMV on {args.openmv_port} @ {args.openmv_baud}')
    except Exception as e:
        print('Failed to open OpenMV port:', e)
        sys.exit(1)

    ard = None
    if not args.dry_run:
        try:
            ard = serial.Serial(args.arduino_port, args.arduino_baud, timeout=0.1)
            print(f'Opened Arduino on {args.arduino_port} @ {args.arduino_baud}')
        except Exception as e:
            print('Failed to open Arduino port:', e)
            print('Running in dry-run mode (no Arduino writes).')
            ard = None

    # Shared state between threads
    state = {
        'dx': None,
        'h': None,
        'cam_ts': 0.0,
        'front_cm': None,
        'side_cm': None,
        'desired_lin': 0.0,
        'desired_ang': 0.0,
    }
    state_lock = threading.Lock()
    stop_event = threading.Event()

    def cam_reader():
        # Continuously read camera lines and update state
        while not stop_event.is_set():
            try:
                line = cam.readline().decode(errors='ignore').strip()
                if not line:
                    time.sleep(0.005)
                    continue
                dx, h = parse_dx_and_height(line)
                with state_lock:
                    state['dx'] = dx
                    state['h'] = h
                    state['cam_ts'] = time.time()
                # optional debug
                # print('CAM:', line)
            except Exception:
                time.sleep(0.01)

    def ard_reader():
        # Read Arduino telemetry and update distances
        while not stop_event.is_set():
            try:
                if ard and ard.in_waiting:
                    tline = ard.readline().decode(errors='ignore').strip()
                    if tline:
                        m_f = re.search(r'USF[:=]?\s*(-?\d+\.?\d*)', tline)
                        m_s = re.search(r'USS[:=]?\s*(-?\d+\.?\d*)', tline)
                        with state_lock:
                            if m_f:
                                try:
                                    state['front_cm'] = float(m_f.group(1))
                                except:
                                    state['front_cm'] = None
                            if m_s:
                                try:
                                    state['side_cm'] = float(m_s.group(1))
                                except:
                                    state['side_cm'] = None
                        print('ARDUINO:', tline)
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.05)

    # PID controller for angular (turning)
    class PID:
        def __init__(self, kp, ki, kd, out_min=-MAX_ANGULAR, out_max=MAX_ANGULAR):
            self.kp = kp
            self.ki = ki
            self.kd = kd
            self.out_min = out_min
            self.out_max = out_max
            self.int = 0.0
            self.last = None

        def update(self, err, dt):
            if self.last is None:
                deriv = 0.0
            else:
                deriv = (err - self.last) / max(dt, 1e-6)
            self.int += err * dt
            out = self.kp * err + self.ki * self.int + self.kd * deriv
            out = max(self.out_min, min(self.out_max, out))
            self.last = err
            return out

    pid = PID(PID_KP, PID_KI, PID_KD)

    # Turning loop: computes desired_ang to center the target
    def turning_loop():
        last_t = time.time()
        while not stop_event.is_set():
            t0 = time.time()
            dt = t0 - last_t if last_t else 1.0 / TURN_RATE_HZ
            last_t = t0
            with state_lock:
                dx = state['dx']
            if dx is None:
                # no detection, zero angular
                desired_ang = 0.0
            else:
                # convert pixel error to normalized [-1,1]
                norm = float(dx) / (IMAGE_WIDTH / 2.0)
                norm = max(-1.0, min(1.0, norm))
                # PID expects error in normalized units (0 at center)
                err = norm
                desired_ang = -pid.update(err, dt)  # negative to turn toward target
            with state_lock:
                state['desired_ang'] = desired_ang
            time.sleep(max(0.0, 1.0 / TURN_RATE_HZ - (time.time() - t0)))

    # Movement loop: computes desired_lin to reach DESIRED_DISTANCE using fused distance
    def movement_loop():
        last_t = time.time()
        while not stop_event.is_set():
            t0 = time.time()
            dt = t0 - last_t if last_t else 1.0 / MOVE_RATE_HZ
            last_t = t0
            with state_lock:
                dx = state['dx']
                h = state['h']
                front = state['front_cm']
            # estimate distance (m) from camera
            cam_dist = estimate_distance_from_height(h) if h is not None and h > 0 else None
            ard_dist = (front / 100.0) if front is not None and front > 0 else None
            # fuse distances: prefer ultrasonic when available (weight 0.8), else camera
            fused = None
            if ard_dist is not None and ard_dist > 0:
                if cam_dist is not None and cam_dist > 0:
                    fused = 0.8 * ard_dist + 0.2 * cam_dist
                else:
                    fused = ard_dist
            elif cam_dist is not None:
                fused = cam_dist

            # Movement policy: only move forward if target is centered (within threshold)
            move = 0.0
            centered = (dx is not None and abs(dx) <= CENTER_THRESHOLD_PX)
            if fused is not None:
                # fused distance available, move until within DESIRED_DISTANCE
                if centered and fused > DESIRED_DISTANCE + 0.05:
                    # P controller toward desired distance
                    err = fused - DESIRED_DISTANCE
                    move = KP_LIN * (err / max(DESIRED_DISTANCE, 0.001)) * MAX_LINEAR
                    move = max(0.0, min(MAX_LINEAR, move))
                else:
                    move = 0.0
            else:
                # no distance measurement; cautiously move only if centered
                if centered:
                    move = 0.1 * MAX_LINEAR
            # safety: if Arduino reports close obstacle, stop
            with state_lock:
                front_now = state['front_cm']
            if front_now is not None and front_now > 0 and front_now < FRONT_OBSTACLE_CM:
                move = 0.0
            with state_lock:
                state['desired_lin'] = move
            time.sleep(max(0.0, 1.0 / MOVE_RATE_HZ - (time.time() - t0)))

    # Sender loop: send combined commands at regular rate
    serial_lock = threading.Lock()
    def sender_loop():
        while not stop_event.is_set():
            t0 = time.time()
            with state_lock:
                lin = state['desired_lin']
                ang = state['desired_ang']
            # send over serial
            msg = f"L:{lin:.3f};A:{ang:.3f}\n"
            if ard:
                try:
                    with serial_lock:
                        ard.write(msg.encode())
                except Exception as e:
                    print('Failed to write to Arduino:', e)
            else:
                print('SEND:', msg.strip())
            time.sleep(max(0.0, 1.0 / SENDER_RATE_HZ - (time.time() - t0)))

    # Start threads
    threads = []
    t_cam = threading.Thread(target=cam_reader, daemon=True)
    t_ard = threading.Thread(target=ard_reader, daemon=True)
    t_turn = threading.Thread(target=turning_loop, daemon=True)
    t_move = threading.Thread(target=movement_loop, daemon=True)
    t_send = threading.Thread(target=sender_loop, daemon=True)
    threads.extend([t_cam, t_ard, t_turn, t_move, t_send])
    for t in threads:
        t.start()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print('Stopping...')
        stop_event.set()
        for t in threads:
            t.join(timeout=0.5)


if __name__ == '__main__':
    main()
