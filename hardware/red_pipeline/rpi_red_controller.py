#!/usr/bin/env python3
"""
rpi_red_controller.py

Runs on Raspberry Pi.
1. Reads "OBJ <seq> <cx> <cy> <area> <ts>" lines from OpenMV (red blob tracking).
2. Runs a PID controller to turn the robot towards the red blob.
3. Sends "T<left>,<right>" commands to the Arduino motor driver.

This script is designed for the high-FPS red-tracking pipeline.
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
IMG_WIDTH = 320  # QVGA width
CAM_FOV_DEG = 60.0  # Approximate camera horizontal field of view

# PID Controller Gains for turning
KP = 1.1  # Proportional gain - Lowered to reduce aggressive reaction
KI = 0.00 # Integral gain - Lowered to prevent overshoot
KD = 14.0  # Derivative gain - Increased to dampen oscillations

# Control Loop Parameters
LOOP_HZ = 20.0  # Target frequency for the control loop (20 Hz = 50ms per loop)
DT = 1.0 / LOOP_HZ
MAX_MOTOR_SPEED = 200  # Max PWM value for motors (0-255)
TURN_SCALING = 2.5     # Scales PID output to motor speed difference - Lowered for less aggressive turns
ANGLE_DEADBAND_DEG = 2.5 # Ignore small angle errors to prevent jitter
INTEGRAL_LIMIT = 150.0   # Prevents integral wind-up
SLEW_RATE_LIMIT = 800.0  # Max change in motor speed per second to smooth motion

# Shared state for sensor data (thread-safe)
state = {
    "cam_x": None,          # Center x-coordinate of the detected blob
    "last_cam_update": 0,   # Timestamp of the last valid camera message
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
                        if msg_type == "OBJ" and len(parts) >= 3:
                            # "OBJ <seq> <cx> <cy> <area> <ts>"
                            state["cam_x"] = int(parts[2])
                            state["last_cam_update"] = time.time()
                        elif msg_type == "NOOBJ":
                            # No object was seen
                            state["cam_x"] = None
        except serial.SerialException as e:
            print(f"OpenMV serial error: {e}. Retrying in 5 seconds...")
            time.sleep(5)
        except Exception as e:
            print(f"An unexpected error occurred in OpenMV reader: {e}")
            time.sleep(1)

# ---------- HELPER FUNCTIONS ----------

def get_angle_from_x(cx):
    """Converts a pixel x-coordinate to an angle in degrees from the center."""
    if cx is None:
        return 0
    # Calculate the offset from the image center
    offset = cx - (IMG_WIDTH / 2.0)
    # Convert pixel offset to angle
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

    # PID state variables
    integral = 0.0
    prev_error = 0.0
    prev_left_motor = 0.0
    prev_right_motor = 0.0

    try:
        while True:
            loop_start_time = time.time()

            # Get a thread-safe copy of the state
            with state_lock:
                cam_x = state["cam_x"]
                last_update = state["last_cam_update"]

            # Safety check: if no camera data for > 0.5s, stop.
            if time.time() - last_update > 0.5:
                cam_x = None

            # --- PID Calculation ---
            error = get_angle_from_x(cam_x)

            # Apply deadband
            if abs(error) < ANGLE_DEADBAND_DEG:
                error = 0
                # Decay integral when in deadband to prevent overshoot
                integral *= 0.9

            # Integral term with anti-windup
            integral += error * DT
            integral = clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)

            # Derivative term
            derivative = (error - prev_error) / DT
            prev_error = error

            # PID output
            pid_output = (KP * error) + (KD * derivative)

            # --- Motor Command Generation ---
            turn_effort = clamp(pid_output * TURN_SCALING, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED)

            # Turn in place
            left_target = -turn_effort
            right_target = turn_effort

            # Apply slew rate limiting for smoother acceleration
            max_change = SLEW_RATE_LIMIT * DT
            left_motor = clamp(left_target, prev_left_motor - max_change, prev_left_motor + max_change)
            right_motor = clamp(right_target, prev_right_motor - max_change, prev_right_motor + max_change)

            prev_left_motor = left_motor
            prev_right_motor = right_motor

            # --- Send Command to Arduino ---
            cmd = f"T{int(left_motor)},{int(right_motor)}\n"
            try:
                arduino.write(cmd.encode('utf-8'))
            except serial.SerialException as e:
                print(f"Arduino write error: {e}")
                # Attempt to reopen port on next loop or exit
                # For now, we just print and continue
                pass

            # --- Debugging Output ---
            print(f"Angle: {error:5.1f}° | PID: {pid_output:6.1f} | Turn: {turn_effort:4.0f} | Motors L/R: {int(left_motor):4d},{int(right_motor):4d}")

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
