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

# --- Config ---
DEFAULT_OPENMV_PORT = '/dev/ttyACM0'
DEFAULT_ARDUINO_PORT = '/dev/ttyUSB0'
OPENMV_BAUD = 115200
ARDUINO_BAUD = 115200

IMAGE_WIDTH = 240  # pixels (used to normalize dx)
MAX_LINEAR = 0.25   # m/s, conservative default
MAX_ANGULAR = 1.0   # rad/s

# Controller gains
KP_ANG = 1.0
KP_LIN = 0.6

# Desired distance: if we can estimate distance (via bounding box) use it, else
# the controller will drive a small forward speed when target is roughly centered.
DESIRED_DISTANCE = 1.0  # meters (used only if the Pi receives a distance estimate)

CAM_TIMEOUT = 0.25  # seconds without camera input -> send zero vel
# Telemetry thresholds
FRONT_OBSTACLE_CM = 25  # stop if object closer than this


def parse_dx_and_height(line: str):
    """Try to extract dx (horizontal offset in pixels) and a height/box metric.
    Returns (dx:int|None, h:int|None). The function is permissive and looks for
    several patterns commonly found in the repo's OpenMV output.
    """
    # dx patterns: "dx=123", "dx:123", "dx 123", or "dx\t123"
    m = re.search(r'dx[:=]?\s*(-?\d+)', line)
    dx = int(m.group(1)) if m else None

    # bounding box height (h or height) or y/h pairs, try common signatures
    m2 = re.search(r'h[:=]?\s*(\d+)', line)
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
    K = 50.0
    return K / float(h)


def compute_cmds(dx, distance):
    # Normalize dx to [-1,1] where 1 means far right edge
    if dx is None:
        norm_dx = 0.0
    else:
        norm_dx = float(dx) / (IMAGE_WIDTH / 2.0)
        norm_dx = max(-1.0, min(1.0, norm_dx))

    # Angular: drive proportional to normalized dx
    ang = -KP_ANG * norm_dx * MAX_ANGULAR

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

    last_cam_time = time.time()
    last_front_dist = None
    last_side_dist = None

    while True:
        try:
            line = cam.readline().decode(errors='ignore').strip()
            now = time.time()

            if line:
                # parse dx and optional bbox height
                dx, h = parse_dx_and_height(line)
                dist = estimate_distance_from_height(h) if h is not None else None
                lin, ang = compute_cmds(dx, dist)
                # Safety override using last known Arduino telemetry (front ultrasonic)
                if last_front_dist is not None and last_front_dist < FRONT_OBSTACLE_CM:
                    lin = 0.0
                    print(f"Obstacle ahead ({last_front_dist:.1f}cm) - overriding linear speed to 0")
                msg = f"L:{lin:.3f};A:{ang:.3f}\n"
                if ard:
                    ard.write(msg.encode())
                print(f'CAM -> "{line}"  =>  {msg.strip()}')
                last_cam_time = now
            else:
                # no new camera data
                if now - last_cam_time > CAM_TIMEOUT:
                    # send stop
                    msg = f"L:0.000;A:0.000\n"
                    if ard:
                        ard.write(msg.encode())
                    # only print occasionally
                    print('No camera data - sending stop')
                    last_cam_time = now

            # read telemetry from Arduino if any
            if ard and ard.in_waiting:
                tline = ard.readline().decode(errors='ignore').strip()
                if tline:
                    # parse ultrasonic telemetry lines like: USF:23.4;USS:45.2
                    m_f = re.search(r'USF[:=]?\s*(-?\d+\.?\d*)', tline)
                    m_s = re.search(r'USS[:=]?\s*(-?\d+\.?\d*)', tline)
                    if m_f:
                        try:
                            last_front_dist = float(m_f.group(1))
                        except:
                            last_front_dist = None
                    if m_s:
                        try:
                            last_side_dist = float(m_s.group(1))
                        except:
                            last_side_dist = None
                    # print raw telemetry for debugging
                    print('ARDUINO:', tline)

        except KeyboardInterrupt:
            print('Exiting')
            break
        except Exception as e:
            print('Error in main loop:', e)
            time.sleep(0.1)


if __name__ == '__main__':
    main()
