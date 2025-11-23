#!/usr/bin/env python3
"""
Pure Pursuit-based replacement for rpi_red_controller.py

Usage: run this script instead of the PID controller. It reads the same
OpenMV "OBJ" and Arduino "US" messages and sends the same "T<left>,<right>\n"
commands to the Arduino motor driver.

This implementation uses the PurePursuit controller in
`hardware/red_pipeline/pure_pursuit.py` to compute steering commands.

It aims to be a drop-in behavioral replacement so you can run either
controller and compare behavior.
"""

import threading
import time
import serial
import sys
import os
import glob
import argparse
import urllib.request
import json

from hardware.red_pipeline.pure_pursuit import PurePursuit, mix_to_wheels

# --------- CONFIG (tunable) ----------
OPENMV_PORT = os.environ.get('OPENMV_PORT', '/dev/ttyACM0')
ARDUINO_PORT = os.environ.get('ARDUINO_PORT', '/dev/ttyUSB0')
BAUD_RATE = 115200

# control loop
LOOP_HZ = 20.0
DT = 1.0 / LOOP_HZ
MAX_MOTOR_SPEED = 220
SLEW_RATE_LIMIT = 3000.0

# distance control defaults
TARGET_DIST_M = 0.45
DIST_DEADBAND_M = 0.1
KP_DIST = 300.0
MIN_DRIVE_SPEED = 120
FWD_SMOOTH_ALPHA = 0.25
DIST_SMOOTH_ALPHA = 0.7
HOLD_US_WHEN_NO_CAM_S = 1.0

# trims
FORWARD_SIGN = -1
MOTOR_LEFT_TRIM = 0
MOTOR_RIGHT_TRIM = -5

# OpenMV message handling
state = {
    'cam_x': None,
    'cam_y': None,
    'cam_area': 0,
    'last_cam_update': 0,
    'us_dist': None,
    'last_us_update': 0,
}
state_lock = threading.Lock()

# Mode polling
MODE_SERVER_URL = None
MODE_POLL_INTERVAL = 1.0
control_mode = 'follow'  # or 'stay'

# Pure Pursuit instance (will be configured via CLI)
pp_controller = PurePursuit()


def autodetect_port(candidate_env: str = None):
    """Autodetect a reasonable serial device (ttyACM / ttyUSB / by-id)"""
    if candidate_env and os.path.exists(candidate_env):
        return candidate_env
    byid = glob.glob('/dev/serial/by-id/*')
    if byid:
        return os.path.realpath(byid[0])
    cands = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    if not cands:
        return None
    try:
        cands = sorted(cands, key=lambda p: os.path.getmtime(p), reverse=True)
    except Exception:
        pass
    return cands[0]


def openmv_reader_thread(openmv_port, baud):
    global state
    print('Starting OpenMV reader thread...')
    while True:
        port = openmv_port
        if not os.path.exists(port):
            candidate = autodetect_port(openmv_port)
            if candidate:
                port = candidate
                print(f'Auto-detected OpenMV port: {port}')
            else:
                print(f'OpenMV port {openmv_port} not found, retrying...')
                time.sleep(2)
                continue
        try:
            ser = serial.Serial(port, baud, timeout=1)
            print(f'OpenMV port {port} opened')
            try:
                while True:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                    parts = line.split()
                    if not parts:
                        continue
                    msg_type = parts[0]
                    with state_lock:
                        if msg_type == 'OBJ' and len(parts) >= 6:
                            try:
                                state['cam_x'] = int(parts[2])
                                state['cam_y'] = int(parts[3])
                                state['cam_area'] = int(parts[4])
                                state['last_cam_update'] = time.time()
                            except Exception:
                                pass
                        elif msg_type == 'NOOBJ':
                            state['cam_x'] = None
                            state['cam_y'] = None
                            state['cam_area'] = 0
            finally:
                try:
                    ser.close()
                except Exception:
                    pass
        except Exception as e:
            print(f'OpenMV serial error: {e} - retrying in 3s')
            time.sleep(3)


def arduino_reader_thread(arduino_port, baud):
    global state
    print('Starting Arduino reader thread...')
    while True:
        try:
            ser = serial.Serial(arduino_port, baud, timeout=1)
            print(f'Arduino port {arduino_port} opened')
            try:
                while True:
                    if ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        if not line:
                            continue
                        parts = line.split()
                        if not parts:
                            continue
                        with state_lock:
                            if parts[0] == 'US' and len(parts) >= 2:
                                try:
                                    state['us_dist'] = float(parts[1])
                                    state['last_us_update'] = time.time()
                                except Exception:
                                    pass
                    else:
                        time.sleep(0.01)
            finally:
                try:
                    ser.close()
                except Exception:
                    pass
        except Exception as e:
            print(f'Arduino serial error: {e} - retrying in 3s')
            time.sleep(3)


def mode_poll_thread(url, interval):
    global control_mode
    print(f'Mode poller: polling {url} every {interval}s')
    while True:
        try:
            req = urllib.request.Request(url, headers={'User-Agent': 'rpi_pp/1.0'})
            with urllib.request.urlopen(req, timeout=2) as resp:
                body = resp.read().decode('utf-8', errors='ignore').strip()
                new = control_mode
                try:
                    j = json.loads(body)
                    if isinstance(j, dict):
                        m = j.get('mode') or j.get('state')
                        if isinstance(m, str) and m.lower() in ('follow', 'stay'):
                            new = m.lower()
                except Exception:
                    lb = body.lower()
                    if 'stay' in lb:
                        new = 'stay'
                    elif 'follow' in lb:
                        new = 'follow'
                if new != control_mode:
                    print(f'Mode change: {control_mode} -> {new}')
                    control_mode = new
        except Exception as e:
            print(f'Mode poll error: {e}')
        time.sleep(interval)


# Helper: convert pixel x to angle deg
def get_angle_from_x(cx, img_width=320, cam_fov_deg=60.0):
    if cx is None:
        return 0.0
    offset = cx - (img_width / 2.0)
    angle = (offset / (img_width / 2.0)) * (cam_fov_deg / 2.0)
    return angle


def clamp(v, a, b):
    return max(a, min(b, v))


def main():
    global pp_controller, MODE_SERVER_URL, MODE_POLL_INTERVAL, control_mode

    parser = argparse.ArgumentParser()
    parser.add_argument('--openmv', help='OpenMV port')
    parser.add_argument('--arduino', help='Arduino port')
    parser.add_argument('--max-pwm', type=int, default=MAX_MOTOR_SPEED)
    parser.add_argument('--lookahead', type=float, default=0.6)
    parser.add_argument('--wheelbase', type=float, default=0.13)
    parser.add_argument('--turn-gain', type=float, default=1.2)
    parser.add_argument('--mode-url', type=str, help='Mode server URL (follow/stay)')
    parser.add_argument('--mode-interval', type=float, default=MODE_POLL_INTERVAL)
    parser.add_argument('--disable-mode', action='store_true', help='Do not poll mode server')
    parser.add_argument('--disable-est', action='store_true', help='Ignore area-based estimation and hold')
    args = parser.parse_args()

    if args.openmv:
        openmv = args.openmv
    else:
        openmv = OPENMV_PORT
    if args.arduino:
        arduino = args.arduino
    else:
        arduino = ARDUINO_PORT

    max_pwm = args.max_pwm

    # configure Pure Pursuit
    pp_controller = PurePursuit(lookahead=args.lookahead, wheelbase=args.wheelbase, turn_gain=args.turn_gain)

    # mode poller
    if args.mode_url:
        MODE_SERVER_URL = args.mode_url
        MODE_POLL_INTERVAL = args.mode_interval
    if MODE_SERVER_URL and not args.disable_mode:
        t_mode = threading.Thread(target=mode_poll_thread, args=(MODE_SERVER_URL, MODE_POLL_INTERVAL), daemon=True)
        t_mode.start()

    # Start readers
    t1 = threading.Thread(target=openmv_reader_thread, args=(openmv, BAUD_RATE), daemon=True)
    t1.start()
    t2 = threading.Thread(target=arduino_reader_thread, args=(arduino, BAUD_RATE), daemon=True)
    t2.start()

    # wait for first camera message
    print('Waiting for first camera message...')
    while True:
        with state_lock:
            if state['last_cam_update'] > 0:
                break
        time.sleep(0.2)
    print('Camera active; starting control loop')

    prev_left = 0.0
    prev_right = 0.0
    prev_fwd = 0.0
    prev_effective_dist = None

    try:
        while True:
            loop_start = time.time()
            with state_lock:
                cam_x = state['cam_x']
                cam_y = state['cam_y']
                cam_area = state['cam_area']
                last_cam = state['last_cam_update']
                us_dist = state['us_dist']
                last_us = state['last_us_update']

            seen_recent = (time.time() - last_cam) <= 0.5
            can_move = (cam_x is not None) and seen_recent

            # mode override
            if MODE_SERVER_URL and control_mode != 'follow':
                can_move = False

            # effective distance: prefer ultrasonic if recent, otherwise hold briefly
            now = time.time()
            effective = None
            if us_dist is not None and (now - last_us) <= 2.0:
                effective = us_dist
            else:
                if (now - last_us) <= HOLD_US_WHEN_NO_CAM_S and last_us > 0:
                    effective = us_dist

            # apply smoothing on distance
            if effective is not None:
                if prev_effective_dist is None:
                    prev_effective_dist = effective
                else:
                    effective = DIST_SMOOTH_ALPHA * prev_effective_dist + (1.0 - DIST_SMOOTH_ALPHA) * effective
                    prev_effective_dist = effective

            # if we don't have an effective distance, don't move
            if effective is None:
                can_move = False

            # compute bearing
            bearing = get_angle_from_x(cam_x)

            # desired forward speed scale based on distance error
            v_scale = 0.0
            if can_move:
                # simple P control to convert distance error into desired speed fraction
                dist_reading = effective
                dist_error = dist_reading - TARGET_DIST_M
                if abs(dist_error) > DIST_DEADBAND_M:
                    fwd = KP_DIST * dist_error
                    if abs(fwd) < MIN_DRIVE_SPEED:
                        fwd = MIN_DRIVE_SPEED if fwd > 0 else -MIN_DRIVE_SPEED
                    # convert PWM-like magnitude to normalized scale in [-1,1]
                    v_scale = clamp(fwd / max_pwm, -1.0, 1.0)
                    # smooth forward
                    v_scale = FWD_SMOOTH_ALPHA * prev_fwd + (1.0 - FWD_SMOOTH_ALPHA) * v_scale
                    prev_fwd = v_scale
                else:
                    v_scale = 0.0
            else:
                v_scale = 0.0

            # Use pure pursuit to get turn command
            if can_move:
                pp_v, pp_turn, info = pp_controller.update(bearing, effective if effective is not None else TARGET_DIST_M, desired_speed_scale=v_scale)
                # The pp.update returns v_scale and turn (both normalized). We will use turn and v_scale.
                final_v = pp_v
                final_turn = pp_turn
            else:
                final_v = 0.0
                final_turn = 0.0

            # Mix to wheel PWMs
            left_pwm, right_pwm = mix_to_wheels(final_v, final_turn, max_pwm=max_pwm)

            # apply trims
            left_pwm = left_pwm + MOTOR_LEFT_TRIM
            right_pwm = right_pwm + MOTOR_RIGHT_TRIM

            # slew rate limit
            max_change = SLEW_RATE_LIMIT * DT
            left = clamp(left_pwm, prev_left - max_change, prev_left + max_change)
            right = clamp(right_pwm, prev_right - max_change, prev_right + max_change)

            prev_left = left
            prev_right = right

            # if cannot move, override to zero
            if not can_move:
                left = 0
                right = 0

            # send to arduino
            cmd = f"T{int(left)},{int(right)}\n"
            try:
                ser = serial.Serial(arduino, BAUD_RATE, timeout=1)
                # write and close immediately to be stateless
                ser.write(cmd.encode('utf-8'))
                ser.close()
            except Exception:
                # fallback: if main arduino reader has opened port, it likely is open in that thread
                pass

            # debug print
            print(f"Seen:{'Y' if can_move else 'N'} | Bearing:{bearing:5.1f}° | Dist:{(effective if effective else 0.0):4.2f}m | v:{final_v:4.2f} t:{final_turn:4.2f} | L/R:{int(left)},{int(right)}")

            # maintain loop
            elapsed = time.time() - loop_start
            sleep_t = DT - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)
    except KeyboardInterrupt:
        print('Shutting down, sending stop')
        try:
            s = serial.Serial(arduino, BAUD_RATE, timeout=1)
            s.write(b'T0,0\n')
            s.close()
        except Exception:
            pass


if __name__ == '__main__':
    main()
