#!/usr/bin/env python3
"""
rpi_red_controller_fixed.py

Runs on Raspberry Pi.
1. Reads "OBJ <seq> <cx> <cy> <area> <ts>" lines from OpenMV (red blob tracking).
2. Runs a PID controller to turn the robot towards the red blob (FIXED: Error sign inverted).
3. Sends "T<left>,<right>" commands to the Arduino motor driver.

This script uses updated, tighter PID tuning for the high-FPS red-tracking pipeline.
"""

import threading
import time
import serial
import sys

# ---------- CONFIGURATION ----------
OPENMV_PORT = "/dev/ttyACM0"   # Serial port for the OpenMV camera
ARDUINO_PORT = "/dev/ttyUSB0"  # Serial port for the Arduino Nano
BAUD_RATE = 115200

# Camera parameters (must match the OpenMV script)
IMG_WIDTH = 320   # QVGA width
IMG_HEIGHT = 240  # QVGA height
CAM_FOV_DEG = 60.0  # Approximate camera horizontal field of view

# PID Controller Gains for turning (TUNING UPDATED)
KP = 0.9   # Proportional gain - higher for quicker response
KI = 0.05  # Integral gain - small value to eliminate steady-state error (bias)
KD = 5.0   # Derivative gain - slightly higher to reduce oscillation

# Control Loop Parameters
LOOP_HZ = 20.0  # Target frequency for the control loop (20 Hz = 50ms per loop)
DT = 1.0 / LOOP_HZ
MAX_MOTOR_SPEED = 200  # Max PWM value for motors (0-255)
TURN_SCALING = 1.6     # Reduced scaling since KP is higher
ANGLE_DEADBAND_DEG = 2.5 # Ignore small angle errors to prevent jitter
INTEGRAL_LIMIT = 150.0   # Prevents integral wind-up
SLEW_RATE_LIMIT = 800.0  # Max change in motor speed per second to smooth motion
ERR_SMOOTH_ALPHA = 0.70  # Slightly more smoothing for cleaner derivative
SMALL_ANGLE_PRIORITIZE_FWD_DEG = 8.0  # Below this, attenuate turning to prefer forward motion

# Distance Control
# Maintain between 0.40 m (min) and 0.80 m (max) by centering target at 0.60 m with +/- 0.20 m deadband
TARGET_DIST_M = 0.6
DIST_DEADBAND_M = 0.2  # deadband -> stop between 0.40 m and 0.80 m
KP_DIST = 120.0       # Slightly stronger forward when far
BASE_SPEED = 0         # Base speed, we'll use P-control for fwd/bwd
MIN_DRIVE_SPEED = 65   # Minimum absolute PWM to overcome static friction when moving

# Drive polarity and trims
# If forward/back looks inverted on your robot, flip FORWARD_SIGN to -1 or 1 accordingly.
FORWARD_SIGN = -1
MOTOR_LEFT_TRIM = 0      # small constant offset to left motor
MOTOR_RIGHT_TRIM = -5    # small negative slows right motor slightly

# Vision gating (only move when we see a blob recently)
DETECTION_TIMEOUT_S = 0.5      # seconds; if no OBJ within this, stop

# Safety stop: if ultrasonic says we're closer than this, hard stop (unless backing up allowed)
MIN_SAFE_DISTANCE_M = 0.40

# Fall detection (object near bottom of frame for sustained time)
FALL_Y_FRACTION = 0.75   # bottom 25% of frame (set 0.66 for bottom third)
FALL_HOLD_S = 10.0       # seconds sustained before alert

# Close-proximity backup behavior
BACKUP_MODE_ENABLED = True
BACKUP_DIST_M = 0.40         # trigger backup at/under this distance (m)
BACKUP_SPEED_PWM = 100       # reverse speed when backing up (PWM)
BACKUP_HEADING_TOL_DEG = 10  # only back up if heading nearly centered
BACKUP_TURN_ATTEN = 0.3      # scale down turn while backing up

# Shared state for sensor data (thread-safe)
state = {
    "cam_x": None,          # Center x-coordinate of the detected blob
    "cam_y": None,          # Center y-coordinate of the detected blob
    "last_cam_update": 0,   # Timestamp of the last valid camera message
    "us_dist": None,        # Distance from the ultrasonic sensor in meters
    "last_us_update": 0,    # Timestamp of the last ultrasonic update
    "fall_since": None,     # timestamp when low-y condition started
    "fall_alerted": False,  # whether alert already printed for current low state
}
state_lock = threading.Lock()

# ---------- SERIAL PORT READER THREAD ----------

def openmv_reader_thread():
    """
    Continuously reads from the OpenMV serial port in a background thread,
    updating the shared state with the latest blob coordinates.
    """
    global state
    print("Starting OpenMV reader thread...")
    while True:
        try:
            with serial.Serial(OPENMV_PORT, BAUD_RATE, timeout=1) as ser:
                print(f"OpenMV port {OPENMV_PORT} opened successfully.")
                while True:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue

                    parts = line.split()
                    if not parts:
                        continue

                    msg_type = parts[0]
                    with state_lock:
                        if msg_type == "OBJ" and len(parts) >= 6:
                            # "OBJ <seq> <cx> <cy> <area> <ts>"
                            try:
                                state["cam_x"] = int(parts[2])
                                state["cam_y"] = int(parts[3])
                                state["cam_area"] = int(parts[4])
                                state["last_cam_update"] = time.time()
                            except ValueError:
                                pass
                        elif msg_type == "NOOBJ":
                            # No object was seen
                            state["cam_x"] = None
                            state["cam_y"] = None
                            state["cam_area"] = 0
        except serial.SerialException as e:
            print(f"OpenMV serial error: {e}. Retrying in 5 seconds...")
            time.sleep(5)
        except Exception as e:
            print(f"An unexpected error occurred in OpenMV reader: {e}")
            time.sleep(1)

def arduino_reader_thread(arduino_ser):
    """
    Continuously reads from the Arduino serial port in a background thread,
    updating the shared state with the latest ultrasonic distance.
    """
    global state
    print("Starting Arduino reader thread...")
    while True:
        try:
            if arduino_ser.in_waiting > 0:
                line = arduino_ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                parts = line.split()
                if not parts:
                    continue

                msg_type = parts[0]
                with state_lock:
                    if msg_type == "US" and len(parts) >= 2:
                        # "US <dist_m>"
                        try:
                            state["us_dist"] = float(parts[1])
                            state["last_us_update"] = time.time()
                        except (ValueError, IndexError):
                            pass # Ignore malformed distance values
        except Exception as e:
            print(f"An unexpected error occurred in Arduino reader: {e}")
            # The main loop handles the serial object lifecycle, so we just wait
            time.sleep(1)

# ---------- HELPER FUNCTIONS ----------

def get_angle_from_x(cx):
    """Converts a pixel x-coordinate to an angle in degrees from the center."""
    if cx is None:
        return 0
    # Calculate the offset from the image center
    offset = cx - (IMG_WIDTH / 2.0)
    # Convert pixel offset to angle
    # Positive angle means object is to the RIGHT of center
    angle = (offset / (IMG_WIDTH / 2.0)) * (CAM_FOV_DEG / 2.0)
    return angle

def clamp(value, min_val, max_val):
    """Clamps a value to a specified range."""
    return max(min_val, min(value, max_val))

# ---------- MAIN CONTROL LOOP ----------

def main():
    """
    Main function to run the PID control loop.
    """
    # Start the OpenMV reader thread
    reader = threading.Thread(target=openmv_reader_thread, daemon=True)
    reader.start()

    # Wait for the first camera message
    print("Waiting for first message from OpenMV...")
    while True:
        with state_lock:
            if state["last_cam_update"] > 0:
                break
        time.sleep(0.5)
    print("First message received. Starting PID controller.")

    # Open the serial port to the Arduino
    try:
        arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
        time.sleep(2) # Wait for Arduino to reset
    except serial.SerialException as e:
        print(f"Fatal: Could not open Arduino port {ARDUINO_PORT}: {e}")
        sys.exit(1)

    # Start the Arduino reader thread (once)
    arduino_reader = threading.Thread(target=arduino_reader_thread, args=(arduino,), daemon=True)
    arduino_reader.start()

    # PID state variables
    integral = 0.0
    prev_error = 0.0
    prev_left_motor = 0.0
    prev_right_motor = 0.0
    error_filt = 0.0

    try:
        while True:
            loop_start_time = time.time()

            # Get a thread-safe copy of the state
            with state_lock:
                cam_x = state["cam_x"]
                cam_y = state["cam_y"]
                last_update = state["last_cam_update"]
                us_dist = state["us_dist"]
                fall_since = state["fall_since"]
                fall_alerted = state["fall_alerted"]

            # Vision gating: must see a blob recently, else STOP
            seen_recently = (time.time() - last_update) <= DETECTION_TIMEOUT_S
            can_move = (cam_x is not None) and seen_recently

            # Determine backup allowance before safety stop
            angle_center_deg = get_angle_from_x(cam_x) if can_move else None
            backup_allowed = False
            if (BACKUP_MODE_ENABLED and can_move and us_dist is not None and us_dist > 0
                and us_dist <= BACKUP_DIST_M and angle_center_deg is not None
                and abs(angle_center_deg) <= BACKUP_HEADING_TOL_DEG):
                backup_allowed = True

            # Safety stop: ultrasonic too close -> hard stop unless backing up allowed
            if us_dist is not None and us_dist > 0 and us_dist < MIN_SAFE_DISTANCE_M and not backup_allowed:
                can_move = False
            
            # --- Fall detection: low vertical position sustained ---
            now = time.time()
            threshold_y = int(IMG_HEIGHT * FALL_Y_FRACTION)
            with state_lock:
                if can_move and cam_y is not None and cam_y >= threshold_y:
                    if state["fall_since"] is None:
                        state["fall_since"] = now
                    elif (not state["fall_alerted"]) and (now - state["fall_since"]) >= FALL_HOLD_S:
                        state["fall_alerted"] = True
                        print("[ALERT] Fall suspected: red target low in frame for >= {:.0f}s".format(FALL_HOLD_S))
                else:
                    # Reset when condition not met or target not seen
                    state["fall_since"] = None
                    state["fall_alerted"] = False
            
            # --- Distance Controller (P-controller) ---
            fwd_bwd_speed = 0
            dist_error = 0
            if backup_allowed:
                # Force reverse to get out of very close proximity
                fwd_bwd_speed = -FORWARD_SIGN * BACKUP_SPEED_PWM
            elif can_move and (us_dist is not None):
                # dist_error > 0 when too far (us_dist > target)
                dist_error = us_dist - TARGET_DIST_M
                # Apply deadband
                if abs(dist_error) > DIST_DEADBAND_M:
                    fwd_bwd_speed = FORWARD_SIGN * KP_DIST * dist_error
                    # Ensure a minimum drive to overcome static friction
                    if abs(fwd_bwd_speed) < MIN_DRIVE_SPEED:
                        fwd_bwd_speed = MIN_DRIVE_SPEED if fwd_bwd_speed > 0 else -MIN_DRIVE_SPEED
            
            # Clamp forward/backward speed
            fwd_bwd_speed = clamp(fwd_bwd_speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED)

            # --- PID Calculation (Turning) ---
            # Convention: angle > 0 when target is to the RIGHT of center.
            # We want positive PID output to produce a RIGHT turn. With the motor mix below
            # (left = fwd + turn, right = fwd - turn), a positive turn increases left and
            # decreases right -> RIGHT turn. Therefore, use non-inverted angle as error.
            error_raw = -get_angle_from_x(cam_x) if can_move else 0.0
            
            # Exponential smoothing to reduce oscillations from noisy centroid
            error_filt = (ERR_SMOOTH_ALPHA * error_filt) + ((1.0 - ERR_SMOOTH_ALPHA) * error_raw)
            error = error_filt


            # Apply deadband
            if abs(error) < ANGLE_DEADBAND_DEG:
                error = 0
                # Decay integral when in deadband to prevent overshoot
                integral *= 0.9

            # Integral term with anti-windup
            if can_move:
                integral += error * DT
                integral = clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
            else:
                integral = 0.0

            # Derivative term
            derivative = (error - prev_error) / DT
            prev_error = error

            # PID output
            pid_output = (KP * error) + (KI * integral) + (KD * derivative)

            # --- Motor Command Generation ---
            turn_effort = clamp(pid_output * TURN_SCALING, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED)

            # Prioritize forward over small turns: attenuate turning for small angle errors
            if can_move:
                ae = abs(error)
                if ae < SMALL_ANGLE_PRIORITIZE_FWD_DEG:
                    # Linear attenuation from 0..1 over 0..threshold
                    att = ae / SMALL_ANGLE_PRIORITIZE_FWD_DEG
                    turn_effort *= att

                # Forward priority under saturation: cap turn so sum with fwd doesn't exceed max
                max_turn_allowed = max(0, MAX_MOTOR_SPEED - abs(fwd_bwd_speed))
                turn_effort = clamp(turn_effort, -max_turn_allowed, max_turn_allowed)

                # While backing up, reduce turning even more
                if backup_allowed:
                    turn_effort *= BACKUP_TURN_ATTEN

            # Combine forward/backward and turning efforts (apply trims only when moving)
            # Positive turn_effort should turn RIGHT (target on the right -> positive error)
            # Using differential mix: left = fwd + turn, right = fwd - turn
            if can_move:
                left_target = (fwd_bwd_speed + turn_effort) + MOTOR_LEFT_TRIM
                right_target = (fwd_bwd_speed - turn_effort) + MOTOR_RIGHT_TRIM
            else:
                left_target = 0
                right_target = 0


            # Apply slew rate limiting for smoother acceleration
            max_change = SLEW_RATE_LIMIT * DT
            left_motor = clamp(left_target, prev_left_motor - max_change, prev_left_motor + max_change)
            right_motor = clamp(right_target, prev_right_motor - max_change, prev_right_motor + max_change)

            prev_left_motor = left_motor
            prev_right_motor = right_motor

            # --- Send Command to Arduino ---
            # If cannot move, send a stop command explicitly
            if not can_move:
                left_motor = 0
                right_motor = 0
            cmd = f"T{int(left_motor)},{int(right_motor)}\n"
            try:
                arduino.write(cmd.encode('utf-8'))
            except serial.SerialException as e:
                print(f"Arduino write error: {e}")
                # Attempt to reopen port on next loop or exit
                # For now, we just print and continue
                pass

            # --- Debugging Output ---
            print(
                f"Seen:{'Y' if can_move else 'N'} | "
                f"RawAngle:{get_angle_from_x(cam_x):5.1f}° | Err:{error:5.1f}° | "
                f"Dist:{(us_dist if us_dist else 0.0):4.2f}m | "
                f"Mode:{'BACKUP' if backup_allowed else 'NORMAL'} | "
                f"Fwd:{fwd_bwd_speed:4.0f} | Turn:{turn_effort:4.0f} | "
                f"L/R:{int(left_motor):4d},{int(right_motor):4d}"
            )

            # Maintain loop frequency
            elapsed_time = time.time() - loop_start_time
            sleep_time = DT - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nShutdown requested. Stopping motors.")
        # Send a final stop command
        try:
            arduino.write(b"T0,0\n")
        except:
            pass
    finally:
        if 'arduino' in locals() and arduino.is_open:
            arduino.close()
        print("Program terminated.")

if __name__ == "__main__":
    main()