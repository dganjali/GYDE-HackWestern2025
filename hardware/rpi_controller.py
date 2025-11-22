# rpi_controller.py
# Run on Raspberry Pi
# pip install pyserial

import threading, time, serial, sys, math

# ---------- CONFIG ----------
OPENMV_PORT = "/dev/ttyACM0"   # set to your OpenMV serial device
ARDUINO_PORT = "/dev/ttyUSB0"  # set to your Arduino serial device
BAUD = 115200

# Camera image width used by OpenMV windowing/crop (set to 240 to match OpenMV windowing)
IMG_WIDTH = 240
CAM_FOV_DEG = 60.0   # approximate horizontal FOV of camera; tune this

# PID constants for heading (turning)
KP = 0.8
KI = 0.0
KD = 0.02

# Distance control
TARGET_DIST_M = 0.5
APPROACH_THRESHOLD_M = 1.0
DIST_TOLERANCE = 0.05

# Fusion rule thresholds
FUSION_MAX_DIFF = 0.35  # if abs(US-CAM) < use US; else decide

# Motor speed limits
MAX_SPEED = 200  # valid range 0-255 (we use -MAX..MAX for sign)
BASE_SPEED = 120

# When True the controller will only turn the robot in place (no forward/back motion).
# Set to False to restore normal distance-based forward/back behavior.
TURN_ONLY = True

# control loop dt
DT = 0.1  # 10 Hz

# -----------------------------
openmv_ser = None
arduino_ser = None

# shared state
state = {
    "cam_x": None,
    "cam_y": None,
    "cam_d": None,
    "us_d": None,
    "last_used_distance": None
}

# ---------- Serial readers ----------
def openmv_reader():
    global openmv_ser, state
    while True:
        try:
            line = openmv_ser.readline().decode('utf-8').strip()
        except Exception as e:
            print("OpenMV read error:", e)
            time.sleep(0.5)
            continue
        if not line:
            continue
        # Expected: OBJ x y d
        parts = line.split()
        if parts and parts[0] == "OBJ":
            try:
                x = int(parts[1])
                y = int(parts[2])
                d = float(parts[3])
            except:
                continue
            state["cam_x"] = x
            state["cam_y"] = y
            state["cam_d"] = d
            #print("OpenMV:", x, y, d)

def arduino_reader():
    global arduino_ser, state
    while True:
        try:
            line = arduino_ser.readline().decode('utf-8').strip()
        except Exception as e:
            print("Arduino read error:", e)
            time.sleep(0.5)
            continue
        if not line:
            continue
        # Expected: US d
        parts = line.split()
        if parts and parts[0] == "US":
            try:
                d = float(parts[1])
            except:
                continue
            state["us_d"] = d
            #print("US:", d)

# ---------- Helpers ----------
def fuse_distance(cam_d, us_d):
    # If one is None, prefer the other
    if cam_d is None and us_d is None:
        return None
    if cam_d is None:
        return us_d
    if us_d is None:
        return cam_d

    # If ultrasonic is close to camera => prefer ultrasonic (usually reliable at close range)
    if abs(us_d - cam_d) <= FUSION_MAX_DIFF:
        state["last_used_distance"] = us_d
        return us_d

    # If ultrasonic >> camera (ultrasound sees farther / maybe different object),
    # and cam_d < us_d by more than threshold, prefer cam (user rule)
    if us_d > cam_d + FUSION_MAX_DIFF:
        state["last_used_distance"] = cam_d
        return cam_d

    # Otherwise prefer ultrasonic as default
    state["last_used_distance"] = us_d
    return us_d

def angle_from_cam_x(cx):
    # cx in pixels, center = IMG_WIDTH/2; map to degrees
    if cx is None or cx < 0:
        return None
    offset_pixels = cx - (IMG_WIDTH / 2.0)
    angle = (offset_pixels / (IMG_WIDTH / 2.0)) * (CAM_FOV_DEG / 2.0)
    return angle  # degrees, positive = target to right

# ---------- Control loop ----------
def control_loop():
    # PID state
    integral = 0.0
    prev_err = 0.0

    while True:
        cam_x = state.get("cam_x")
        cam_d = state.get("cam_d")
        us_d = state.get("us_d")

        fused = fuse_distance(cam_d, us_d)
        angle_deg = angle_from_cam_x(cam_x)

        # Heading PID only uses camera angle (if available). If camera is not seeing, try to stop turning.
        if angle_deg is None:
            # No visual, don't turn, maybe stop motors
            turn_output = 0.0
        else:
            # PID on angle (want angle -> 0)
            err = -angle_deg  # negative because if target is right (positive), we need positive turn to right
            integral += err * DT
            derivative = (err - prev_err) / DT
            turn_output = KP * err + KI * integral + KD * derivative
            prev_err = err

        # Convert turn_output (deg) to motor differential
        # small mapping: max turn corresponds to MAX_SPEED
        TURN_SCALE = 3.0  # tune (deg -> motor speed)
        diff = int(max(-MAX_SPEED, min(MAX_SPEED, turn_output * TURN_SCALE)))

        # Distance control (normally used to compute forward/back speed)
        move_speed = 0
        if not TURN_ONLY:
            if fused is None:
                move_speed = 0
            else:
                if fused > APPROACH_THRESHOLD_M:
                    # target far: approach until TARGET_DIST_M
                    # simple proportional: more error => more forward speed
                    err_dist = fused - TARGET_DIST_M
                    # scale into speed
                    KP_DIST = 180.0
                    move_speed = int(max(0, min(MAX_SPEED, KP_DIST * err_dist)))
                elif fused > TARGET_DIST_M + DIST_TOLERANCE:
                    # small correction forward
                    err_dist = fused - TARGET_DIST_M
                    KP_DIST = 150.0
                    move_speed = int(max(0, min(MAX_SPEED, KP_DIST * err_dist)))
                elif fused < TARGET_DIST_M - DIST_TOLERANCE:
                    # too close -> back off
                    err_dist = (TARGET_DIST_M - fused)
                    KP_BACK = 150.0
                    move_speed = -int(max(0, min(MAX_SPEED, KP_BACK * err_dist)))
                else:
                    move_speed = 0
        else:
            # TURN_ONLY mode: force zero forward/back motion
            move_speed = 0

        # Combine move_speed and diff into left/right motor commands (tank)
        if TURN_ONLY:
            # Turn in place: left and right are opposite to rotate robot
            left = -diff
            right = diff
        else:
            left = move_speed - diff
            right = move_speed + diff

        # clamp
        left = max(-255, min(255, left))
        right = max(-255, min(255, right))

        # Send tank command: "T<left>,<right>\n"
        cmd = "T{:+d},{:+d}\n".format(int(left), int(right))
        try:
            arduino_ser.write(cmd.encode('utf-8'))
        except Exception as e:
            print("Write to Arduino error:", e)

        # debug print
        print("Fused: {:.2f}m Cam:{:.2f} US:{:.2f} | angle:{:.1f}deg | L,R={} {}".format(
            fused if fused else -1.0,
            cam_d if cam_d else -1.0,
            us_d if us_d else -1.0,
            angle_deg if angle_deg else 0.0,
            left, right
        ))

        time.sleep(DT)

# ---------- Main ----------
def main():
    global openmv_ser, arduino_ser
    try:
        openmv_ser = serial.Serial(OPENMV_PORT, BAUD, timeout=1)
    except Exception as e:
        print("OpenMV serial open failed:", e)
        sys.exit(1)

    try:
        arduino_ser = serial.Serial(ARDUINO_PORT, BAUD, timeout=1)
    except Exception as e:
        print("Arduino serial open failed:", e)
        sys.exit(1)

    # start threads
    t1 = threading.Thread(target=openmv_reader, daemon=True)
    t2 = threading.Thread(target=arduino_reader, daemon=True)
    t1.start(); t2.start()

    # wait a little for serials to settle
    time.sleep(1.0)
    # start control loop in main thread
    control_loop()

if __name__ == "__main__":
    main()
