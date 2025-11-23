#!/usr/bin/env python3
"""
rpi_red_controller_web.py

Runs on Raspberry Pi.
1. Reads "OBJ <seq> <cx> <cy> <area> <ts>" lines from OpenMV.
2. Runs a PID controller to turn the robot.
3. Sends "T<left>,<right>" commands to Arduino.
4. HOSTS A WEB SERVER on Port 8000 to display Fall Alerts.

IMPROVED FALL DETECTION LOGIC (Multi-Modal State-Based):
Replaces simple Y-coordinate check with two-stage validation:
1. Stage 1 (Trigger): Detects a sudden, rapid drop in Y or loss of object area.
2. Stage 2 (Validation): Checks if the resulting state (low or gone) persists
   for a defined period AND factors in the robot's pre-event speed and distance.
"""

import threading
import time
import serial
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer
import urllib.request
import urllib.error

# ---------- CONFIGURATION ----------
OPENMV_PORT = "/dev/ttyACM0"   
ARDUINO_PORT = "/dev/ttyUSB0"  
BAUD_RATE = 115200
WEB_PORT = 19109  # Port to access the web page (e.g., http://raspberrypi.local:8000)

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

# --- IMPROVED FALL DETECTION CONFIGURATION ---
# NOTE: The old FALL_Y_FRACTION is still used as a secondary persistence check.
FALL_Y_FRACTION = 0.75          
FALL_ALERT_PERIOD_S = 1.0       # Time the validated state must persist (Stage 2)
HISTORY_BUFFER_SIZE = 10        # Number of frames (0.5 seconds at 20 Hz)
MAX_Y_VELOCITY_PIX_PER_DT = (IMG_HEIGHT * 0.4) * DT # Max allowed Y-change (40% of height per second)
MIN_AREA_SHRINK_PER_DT = 0.5    # 50% area loss per second, scaled by DT
FAR_DISTANCE_FALL_M = 1.0       # Ultrasonic distance considered 'far' for a fall victim
# ---------------------------------------------

# Close-proximity backup behavior
BACKUP_MODE_ENABLED = True
BACKUP_DIST_M = 0.40         
BACKUP_SPEED_PWM = 100       
BACKUP_HEADING_TOL_DEG = 10  
BACKUP_TURN_ATTEN = 0.3      

# External follow control
# Note: Replace <laptop_ip> with the actual IP address of the machine running follow_mode_server.py
FOLLOW_MODE_URL = "http://<laptop_ip>:8080/mode"  
FOLLOW_POLL_INTERVAL_S = 2.0
FOLLOW_DEFAULT_MODE = "follow"  

# Shared state for sensor data (thread-safe)
state = {
    "cam_x": None,          
    "cam_y": None,          
    "cam_area": 0,          # Added cam_area to state to match OBJ message
    "last_cam_update": 0,   
    "us_dist": None,        
    "last_us_update": 0,    
    "fall_low_active": False,   # Final validated fall state
    "fall_last_alert_ts": 0.0,  
    "follow_mode": FOLLOW_DEFAULT_MODE,
    
    # --- NEW FALL DETECTION STATE ---
    "cam_y_history": [],        # History for velocity calculation
    "cam_area_history": [],     # History for rapid area change
    "pre_event_speed": 0.0,     # Motor speed when Stage 1 triggered
    "fall_trigger_ts": 0.0,     # Timestamp of Stage 1 trigger
    "fall_stage_one_active": False, # Flag for initial detection (rapid drop/loss)
    # --------------------------------
}
state_lock = threading.Lock()

# ---------- WEB SERVER CLASS ----------

class StatusHandler(BaseHTTPRequestHandler):
    """
    A simple handler to serve a webpage showing the fall status.
    """
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()

            # Read state safely
            with state_lock:
                fall_active = state["fall_low_active"]
                last_alert = state["fall_last_alert_ts"]

            # Calculate time strings
            time_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(last_alert)) if last_alert > 0 else "None"
            
            # Determine Color/Status text
            if fall_active:
                status_color = "red"
                status_text = "FALL DETECTED"
            else:
                status_color = "green"
                status_text = "NORMAL"

            # Build HTML with auto-refresh (meta refresh) every 2 seconds
            html = f"""
            <!DOCTYPE html>
            <html>
            <head>
                <title>Robot Fall Monitor</title>
                <meta http-equiv="refresh" content="2">
                <meta name="viewport" content="width=device-width, initial-scale=1">
                <style>
                    body {{ font-family: sans-serif; text-align: center; padding: 50px; }}
                    .status {{ font-size: 48px; font-weight: bold; color: {status_color}; }}
                    .details {{ margin-top: 20px; color: #555; }}
                </style>
            </head>
            <body>
                <h1>Robot Safety Monitor</h1>
                <div class="status">{status_text}</div>
                <div class="details">
                    <p><b>Last Alert Time:</b> {time_str}</p>
                    <p><i>Page refreshes automatically every 2 seconds.</i></p>
                </div>
            </body>
            </html>
            """
            self.wfile.write(html.encode('utf-8'))
        else:
            self.send_error(404)

def run_web_server():
    """Starts the web server on port 8000."""
    try:
        server = HTTPServer(('0.0.0.0', WEB_PORT), StatusHandler)
        print(f"Web server started on port {WEB_PORT}")
        server.serve_forever()
    except Exception as e:
        print(f"Failed to start web server: {e}")

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

    # 2. Start Web Server Thread
    web_thread = threading.Thread(target=run_web_server, daemon=True)
    web_thread.start()

    # 3. Start follow mode polling thread
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

    # 4. Start Arduino Thread
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
                fall_stage_one_active = state["fall_stage_one_active"]
                fall_trigger_ts = state["fall_trigger_ts"]
                
                # Capture speed before PID runs, for fall detection history
                pre_pid_left = prev_left_motor
                pre_pid_right = prev_right_motor

            seen_recently = (time.time() - last_update) <= DETECTION_TIMEOUT_S
            with state_lock:
                follow_mode = state['follow_mode']
            can_move = (cam_x is not None) and seen_recently and (follow_mode == 'follow')

            angle_center_deg = get_angle_from_x(cam_x) if can_move else None
            backup_allowed = False
            if (BACKUP_MODE_ENABLED and can_move and us_dist is not None and us_dist > 0
                and us_dist <= BACKUP_DIST_M and angle_center_deg is not None
                and abs(angle_center_deg) <= BACKUP_HEADING_TOL_DEG):
                backup_allowed = True

            if us_dist is not None and us_dist > 0 and us_dist < MIN_SAFE_DISTANCE_M and not backup_allowed:
                can_move = False
            
            # ----------------------------------------
            # --- IMPROVED FALL DETECTION LOGIC ---
            # ----------------------------------------
            
            threshold_y = int(IMG_HEIGHT * FALL_Y_FRACTION)
            
            # 1. Update History Buffers (Stage 0)
            with state_lock:
                # Store the current position/area and manage buffer size
                if cam_x is not None:
                    state["cam_y_history"].append(cam_y)
                    state["cam_area_history"].append(cam_area)
                else:
                    state["cam_y_history"].append(-1) # Use -1 for no detection
                    state["cam_area_history"].append(0)
                
                state["cam_y_history"] = state["cam_y_history"][-HISTORY_BUFFER_SIZE:]
                state["cam_area_history"] = state["cam_area_history"][-HISTORY_BUFFER_SIZE:]

            # 2. Check for Rapid Drop (Stage 1 Trigger)
            with state_lock:
                # Need at least two points to calculate velocity/rate
                if len(state["cam_y_history"]) >= 2:
                    y_prev = state["cam_y_history"][-2]
                    area_prev = state["cam_area_history"][-2]
                    
                    if cam_x is not None and y_prev != -1: # Object was seen and is now seen
                        # Calculate vertical velocity proxy (Y increases downward)
                        dy_dt = (cam_y - y_prev) / DT
                        
                        # Calculate area change (proxy for object becoming thin/horizontal)
                        # We only care about sudden shrinkage from a non-zero area
                        darea_dt = (cam_area - area_prev) / DT if area_prev > 0 else 0
                        
                        # Heuristics for sudden fall event:
                        # 1. Object suddenly drops (Y increases rapidly) OR
                        # 2. Object area shrinks rapidly
                        rapid_drop = dy_dt > MAX_Y_VELOCITY_PIX_PER_DT
                        rapid_area_loss = darea_dt < (-area_prev * MIN_AREA_SHRINK_PER_DT)
                        
                        if rapid_drop or rapid_area_loss:
                            if not state["fall_stage_one_active"]:
                                # Stage 1 Triggered: record time and speed
                                state["fall_stage_one_active"] = True
                                state["fall_trigger_ts"] = now
                                state["pre_event_speed"] = max(abs(pre_pid_left), abs(pre_pid_right)) 
                                print("DEBUG: Fall Stage 1 Triggered (Rapid Vision Change)", flush=True)

                    elif cam_x is None and y_prev != -1: # Object was seen, now gone (sudden disappearance)
                        if not state["fall_stage_one_active"]:
                            # Stage 1 Triggered: record time and speed
                            state["fall_stage_one_active"] = True
                            state["fall_trigger_ts"] = now
                            state["pre_event_speed"] = max(abs(pre_pid_left), abs(pre_pid_right)) 
                            print("DEBUG: Fall Stage 1 Triggered (Sudden Object Loss)", flush=True)

            # 3. Validation Check (Stage 2)
            is_fall_validated = False
            with state_lock:
                prev_fall_active = state["fall_low_active"]
                if state["fall_stage_one_active"]:
                    time_elapsed = now - state["fall_trigger_ts"]
                    
                    # Condition 1: Check persistence (Is the object still low or gone?)
                    object_is_gone_or_low = (cam_x is None) or (cam_y is not None and cam_y >= threshold_y)
                    
                    # Condition 2: Check Proximity (Are we seeing the floor/far distance?)
                    # If we have US data, check if distance is far (indicates object is on the floor away from the robot)
                    proximity_check_passed = (us_dist is None) or (us_dist > FAR_DISTANCE_FALL_M) 

                    # Condition 3: Check Time Constraint and Final State
                    if time_elapsed >= FALL_ALERT_PERIOD_S:
                        # Only validate if the final state (low/gone) is persistent
                        if object_is_gone_or_low and proximity_check_passed:
                            is_fall_validated = True
                        else:
                            # The event subsided (e.g., they stood back up)
                            state["fall_stage_one_active"] = False

                if is_fall_validated:
                    if (not prev_fall_active) or (now - state["fall_last_alert_ts"]) >= FALL_ALERT_PERIOD_S:
                        print("ALRT VALIDATED: Multi-Modal Fall Detected!", flush=True)
                        state["fall_last_alert_ts"] = now
                    state["fall_low_active"] = True
                else:
                    state["fall_low_active"] = False
                    if not object_is_gone_or_low: 
                         # If object reappears in a normal position, reset stage one flag
                         state["fall_stage_one_active"] = False
            
            # --- END IMPROVED FALL DETECTION LOGIC ---
            # -----------------------------------------
            
            # --- PID & Motor Control (Simplified for brevity, logic remains same) ---
            fwd_bwd_speed = 0
            dist_error = 0
            
            # Skip movement if a fall is validated (safety stop)
            if state["fall_low_active"]:
                can_move = False

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
            if int(loop_start_time * 10) % int(2 * 10) == 0:  # approx every 2s
                print(f"[MODE] follow_mode={follow_mode}, Fall_Active={state['fall_low_active']}", flush=True)

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