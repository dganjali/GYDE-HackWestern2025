#!/usr/bin/env python3
"""
serial_bridge.py

Simple bridge: read camera (OpenMV) serial lines, parse detection, send compact dx/dy/F lines
to an Arduino. Also prints ultrasonic lines received from Arduino.

Adapted from example/arduino_serial.py but tuned to 115200 and to include US parsing.
"""

import serial
import time

CAM_PORT = '/dev/ttyACM0'
CAM_BAUD = 115200
ARDUINO_PORT = '/dev/ttyUSB0'
ARDUINO_BAUD = 115200

time.sleep(0.5)

try:
    cam = serial.Serial(CAM_PORT, CAM_BAUD, timeout=0.1)
except Exception as e:
    print('Failed to open camera port:', e)
    raise

try:
    ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.1)
except Exception as e:
    print('Failed to open arduino port:', e)
    raise

time.sleep(1.0)

last_cam_time = time.time()
cam_timeout = 0.2

while True:
    now = time.time()

    # read camera
    if cam.in_waiting:
        line = cam.readline().decode(errors='ignore').strip()
        if not line:
            continue
        print('CAM:', line)

        # camera may output our new OBJ format or older dx;dy;F; format
        if 'OBJ' in line:
            # ignore here; bridge can be used for dx;dy format
            pass
        if 'dx:' in line and 'dy:' in line:
            try:
                parts = line.split(';')
                dx = int(parts[0].split(':')[1])
                dy = int(parts[1].split(':')[1])
                az = -dx
                el = dy
                fire = 1 if abs(dx) < 5 and abs(dy) < 5 else 0
                msg = f"dx:{az};dy:{el};F:{fire}\n"
                ser.write(msg.encode())
                print('-> ARDUINO:', msg.strip())
                last_cam_time = now
            except Exception as e:
                print('Parse camera error:', e)

    # send default if camera silent
    elif now - last_cam_time > cam_timeout:
        msg = 'dx:0;dy:0;F:0\n'
        ser.write(msg.encode())
        last_cam_time = now

    # read Arduino messages (including US)
    if ser.in_waiting:
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            # show ultrasonic lines specially
            if line.startswith('US '):
                print('ARDUINO US:', line[3:])
            else:
                print('ARDUINO:', line)

    time.sleep(0.01)
