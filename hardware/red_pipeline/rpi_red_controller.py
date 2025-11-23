#!/usr/bin/env python3
"""
rpi_red_controller.py

Runs on Raspberry Pi.
1. Reads "OBJ <seq> <cx> <cy> <area> <ts>" lines from OpenMV.
2. Runs a PID controller to turn the robot.
3. Sends "T<left>,<right>" commands to Arduino.
4. Polls a web server to determine if it should be in "follow" or "stay" mode.
"""

import threading
import time
import serial
import sys
import urllib.request
import urllib.error

# ---------- CONFIGURATION ----------
OPENMV_PORT = "/dev/ttyACM0"    
ARDUINO_PORT = "/dev/ttyUSB0"   
BAUD_RATE = 115200

# Camera parameters
IMG_WIDTH = 320    
IMG_HEIGHT = 240   
CAM_FOV_DEG = 60.0   

# PID Controller Gains
KP = 0.9    
KI = 0.05  
KD = 5.0    

# Control Loop Parameters
LOOP_HZ = 20.0   
DT = 1.0 / LOOP_HZ
MAX_MOTOR_SPEED = 200   
TURN_SCALING = 1.6    
ANGLE_DEADBAND_DEG = 2.5 
INTEGRAL_LIMIT = 150.0    
SLEW_RATE_LIMIT = 800.0   
ERR_SMOOTH_ALPHA = 0.70   
SMALL_ANGLE_PRIORITIZE_FWD_DEG = 8.0   

# Distance Control
TARGET_DIST_M = 0.6
DIST_DEADBAND_M = 0.2   
KP_DIST = 120.0          
MIN_DRIVE_SPEED = 65    

# Drive polarity and trims
FORWARD_SIGN = -1
MOTOR_LEFT_TRIM = 0      
MOTOR_RIGHT_TRIM = -5    

# Vision gating
DETECTION_TIMEOUT_S = 0.5      

# Safety stop
MIN_SAFE_DISTANCE_M = 0.40

# (Fall detection removed)

# Close-proximity backup behavior
BACKUP_MODE_ENABLED = True
BACKUP_DIST_M = 0.40           
BACKUP_SPEED_PWM = 100         
BACKUP_HEADING_TOL_DEG = 10   
BACKUP_TURN_ATTEN = 0.3      

# External follow control
FOLLOW_MODE_URL = "http://localhost:8080/mode"  
FOLLOW_POLL_INTERVAL_S = 2.0
FOLLOW_DEFAULT_MODE = "follow"   

# Shared state for sensor data (thread-safe)
state = {
    "cam_x": None,         
    "cam_y": None,         
    "cam_area": 0,         # Added cam_area to state to match OBJ message
    "last_cam_update": 0,  
    "us_dist": None,       
    "last_us_update": 0,   
    "follow_mode": FOLLOW_DEFAULT_MODE,
}
state_lock = threading.Lock()

# (Web server for fall alerts removed)

# ---------- SERIAL PORT READER THREADS ----------

def openmv_reader_thread():
    global state
    print("Starting OpenMV reader thread...")
    while True:
        try:
            with serial.Serial(OPENMV_PORT, BAUD_RATE, timeout=1) as ser:
                print(f"OpenMV port {OPENMV_PORT} opened successfully.")
                while True:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line: continue

                    parts = line.split()
                    if not parts: continue

                    msg_type = parts[0]
                    with state_lock:
                        if msg_type == "OBJ" and len(parts) >= 6:
                            try:
                                state["cam_x"] = int(parts[2])
                                state["cam_y"] = int(parts[3])
                                state["cam_area"] = int(parts[4])
                                state["last_cam_update"] = time.time()
                            except ValueError: pass
                        elif msg_type == "NOOBJ":
                            state["cam_x"] = None
                            state["cam_y"] = None
                            state["cam_area"] = 0
        except serial.SerialException:
            time.sleep(5)
        except Exception:
            time.sleep(1)

def follow_mode_poll_thread():
    """Background thread to poll remote server for follow/stay mode."""
    print(f"Follow mode polling thread started (interval={FOLLOW_POLL_INTERVAL_S}s, url={FOLLOW_MODE_URL})")
    while True:
        new_mode = None
        try:
            with urllib.request.urlopen(FOLLOW_MODE_URL, timeout=2) as resp:
                txt = resp.read().decode('utf-8', errors='ignore').strip().lower()
                if 'follow' in txt:
                    new_mode = 'follow'
                elif 'stay' in txt:
                    new_mode = 'stay'
        except Exception:
            # Network or parse error; keep previous mode
            pass
        if new_mode:
            with state_lock:
                state['follow_mode'] = new_mode
        time.sleep(FOLLOW_POLL_INTERVAL_S)

def arduino_reader_thread(arduino_ser):
    global state
    print("Starting Arduino reader thread...")
    while True:
        try:
            if arduino_ser.in_waiting > 0:
                line = arduino_ser.readline().decode('utf-8', errors='ignore').strip()
                if not line: continue
                parts = line.split()
                if not parts: continue
                
                msg_type = parts[0]
                with state_lock:
                    if msg_type == "US" and len(parts) >= 2:
                        try:
                            state["us_dist"] = float(parts[1])
                            state["last_us_update"] = time.time()
                        except (ValueError, IndexError): pass 
        except Exception:
            time.sleep(1)

# ---------- HELPER FUNCTIONS ----------

def get_angle_from_x(cx):
    if cx is None: return 0
    offset = cx - (IMG_WIDTH / 2.0)
    angle = (offset / (IMG_WIDTH / 2.0)) * (CAM_FOV_DEG / 2.0)
    return angle

def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

# ---------- MAIN CONTROL LOOP ----------

def main():
    # 1. Start OpenMV Thread
    reader = threading.Thread(target=openmv_reader_thread, daemon=True)
    reader.start()

    # 2. Start follow mode polling thread
    follow_thread = threading.Thread(target=follow_mode_poll_thread, daemon=True)
    follow_thread.start()

    print("Waiting for first message from OpenMV...")
    while True:
        with state_lock:
            if state["last_cam_update"] > 0: break
        time.sleep(0.5)
    print("First message received. Starting PID controller.")

    try:
        arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
        time.sleep(2) 
    except serial.SerialException as e:
        print(f"Fatal: Could not open Arduino port {ARDUINO_PORT}: {e}")
        sys.exit(1)

    # 3. Start Arduino Thread
    arduino_reader = threading.Thread(target=arduino_reader_thread, args=(arduino,), daemon=True)
    arduino_reader.start()

    integral = 0.0
    prev_error = 0.0
    prev_left_motor = 0.0
    prev_right_motor = 0.0
    error_filt = 0.0

    try:
        while True:
            loop_start_time = time.time()
            now = loop_start_time # Use loop start time for consistency

            with state_lock:
                cam_x = state["cam_x"]
                cam_y = state["cam_y"]
                cam_area = state["cam_area"]
                last_update = state["last_cam_update"]
                us_dist = state["us_dist"]

            seen_recently = (time.time() - last_update) <= DETECTION_TIMEOUT_S
            with state_lock:
                follow_mode = state['follow_mode']
            
            # can_move starts as true if object is seen and we are in follow mode
            can_move = (cam_x is not None) and seen_recently and (follow_mode == 'follow')

            angle_center_deg = get_angle_from_x(cam_x) if can_move else None
            backup_allowed = False
            if (BACKUP_MODE_ENABLED and can_move and us_dist is not None and us_dist > 0
                and us_dist <= BACKUP_DIST_M and angle_center_deg is not None
                and abs(angle_center_deg) <= BACKUP_HEADING_TOL_DEG):
                backup_allowed = True

            # CHECK 1: MINIMUM SAFE DISTANCE CHECK (The most likely culprit for stopping)
            if us_dist is not None and us_dist > 0 and us_dist < MIN_SAFE_DISTANCE_M and not backup_allowed:
                can_move = False
                # Added debug print to confirm safety stop is the cause
                print(f"DEBUG: Safety Stop Triggered! us_dist={us_dist:.2f}m < {MIN_SAFE_DISTANCE_M:.2f}m", flush=True)

            
            # (Fall detection logic removed)
            
            # --- PID & Motor Control (Simplified for brevity, logic remains same) ---
            fwd_bwd_speed = 0
            dist_error = 0
            
            # **FIXED: Fall detection safety stop removed here!**
            # (The block `if state["fall_low_active"]: can_move = False` has been deleted)

            if backup_allowed and follow_mode == 'follow':
                fwd_bwd_speed = -FORWARD_SIGN * BACKUP_SPEED_PWM
            elif can_move and (us_dist is not None):
                dist_error = us_dist - TARGET_DIST_M
                if abs(dist_error) > DIST_DEADBAND_M:
                    fwd_bwd_speed = FORWARD_SIGN * KP_DIST * dist_error
                    if abs(fwd_bwd_speed) < MIN_DRIVE_SPEED:
                        fwd_bwd_speed = MIN_DRIVE_SPEED if fwd_bwd_speed > 0 else -MIN_DRIVE_SPEED
            
            fwd_bwd_speed = clamp(fwd_bwd_speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED)

            error_raw = -get_angle_from_x(cam_x) if can_move else 0.0
            error_filt = (ERR_SMOOTH_ALPHA * error_filt) + ((1.0 - ERR_SMOOTH_ALPHA) * error_raw)
            error = error_filt

            if abs(error) < ANGLE_DEADBAND_DEG:
                error = 0
                integral *= 0.9

            if can_move:
                integral += error * DT
                integral = clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
            else:
                integral = 0.0

            derivative = (error - prev_error) / DT
            prev_error = error

            pid_output = (KP * error) + (KI * integral) + (KD * derivative)
            turn_effort = clamp(pid_output * TURN_SCALING, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED)

            if can_move:
                ae = abs(error)
                if ae < SMALL_ANGLE_PRIORITIZE_FWD_DEG:
                    att = ae / SMALL_ANGLE_PRIORITIZE_FWD_DEG
                    turn_effort *= att
                max_turn_allowed = max(0, MAX_MOTOR_SPEED - abs(fwd_bwd_speed))
                turn_effort = clamp(turn_effort, -max_turn_allowed, max_turn_allowed)
                if backup_allowed:
                    turn_effort *= BACKUP_TURN_ATTEN

            if can_move:
                left_target = (fwd_bwd_speed + turn_effort) + MOTOR_LEFT_TRIM
                right_target = (fwd_bwd_speed - turn_effort) + MOTOR_RIGHT_TRIM
            else:
                left_target = 0
                right_target = 0

            max_change = SLEW_RATE_LIMIT * DT
            left_motor = clamp(left_target, prev_left_motor - max_change, prev_left_motor + max_change)
            right_motor = clamp(right_target, prev_right_motor - max_change, prev_right_motor + max_change)

            prev_left_motor = left_motor
            prev_right_motor = right_motor

            if not can_move:
                left_motor = 0
                right_motor = 0
            
            # Command the Arduino
            cmd = f"T{int(left_motor)},{int(right_motor)}\n"
            try:
                arduino.write(cmd.encode('utf-8'))
            except serial.SerialException: pass

            # Loop timing
            elapsed_time = time.time() - loop_start_time
            sleep_time = DT - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)

            # Periodic status print for mode
            if int(loop_start_time * 10) % int(2 * 10) == 0: # approx every 2s
                print(f"[MODE] follow_mode={follow_mode}, Can_Move={can_move}", flush=True)

    except KeyboardInterrupt:
        print("\nShutdown requested.")
        try: arduino.write(b"T0,0\n")
        except: pass
    finally:
        if 'arduino' in locals() and arduino.is_open:
            arduino.close()
        print("Program terminated.")

if __name__ == "__main__":
    main()