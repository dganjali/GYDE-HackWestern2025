#!/usr/bin/env python3
"""
pi_motor_control.py

Interactive test utility to control the Arduino motor controller from the Pi.
Sends commands in the format expected by `robot_nano.ino`: "T<left>,<right>\n"

Usage:
  python3 hardware/pi_motor_control.py --port /dev/ttyUSB0

Commands (type at prompt):
  tank L R      - send signed motor values L and R (integers)
  raw L R       - same as tank
  forward S     - both motors forward at speed S (0-255)
  back S        - both motors backward at speed S (0-255)
  left S        - rotate left in place, S determines turn speed
  right S       - rotate right in place
  stop          - stop motors (sends STOP)
  exit / quit   - exit program

This script also prints lines received from the Arduino (e.g., ultrasonic readings)
so you can verify two-way communication.
"""

import argparse
import threading
import time
import sys

try:
    import serial
except Exception:
    print("pyserial is required. Install with: pip3 install pyserial")
    raise


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


def reader_thread(ser):
    while True:
        try:
            line = ser.readline()
            if not line:
                continue
            # decode if bytes
            try:
                text = line.decode('utf-8', errors='replace').strip()
            except Exception:
                text = str(line)
            # Print on its own line and re-print prompt to avoid corrupting user input
            try:
                sys.stdout.write("\n[ARDUINO] " + text + "\n> ")
                sys.stdout.flush()
            except Exception:
                # fallback
                print("[ARDUINO]", text)
        except Exception as e:
            # Print error on its own line and re-print prompt
            try:
                sys.stdout.write("\nSerial read error: {}\n> ".format(e))
                sys.stdout.flush()
            except Exception:
                print("Serial read error:", e)
            time.sleep(0.5)


def send_tank(ser, left, right):
    left = clamp(left)
    right = clamp(right)
    cmd = "T{:+d},{:+d}\n".format(int(left), int(right))
    try:
        ser.write(cmd.encode('utf-8'))
    except Exception:
        # some setups accept str write
        try:
            ser.write(cmd)
        except Exception as e:
            print("Write error:", e)
    print("[SENT]", cmd.strip())


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', '-p', default='/dev/ttyUSB0', help='Arduino serial port')
    p.add_argument('--baud', '-b', default=115200, type=int, help='baud rate')
    args = p.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except Exception as e:
        print("Failed to open serial port {}: {}".format(args.port, e))
        sys.exit(1)

    t = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    t.start()

    print("Connected to {} at {} baud".format(args.port, args.baud))
    print("Type 'help' for commands. 'exit' to quit.")

    try:
        while True:
            try:
                line = input('> ').strip()
            except EOFError:
                break
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()
            if cmd in ('exit', 'quit'):
                break
            if cmd == 'help':
                print(__doc__)
                continue
            if cmd in ('stop', 's'):
                try:
                    ser.write(b"STOP\n")
                except Exception:
                    try:
                        ser.write("STOP\n")
                    except Exception as e:
                        print("Write error:", e)
                print("[SENT] STOP")
                continue
            # Single-letter shortcuts
            if cmd == 'l':
                # left in place at default speed (120)
                s = 120
                send_tank(ser, -s, s)
                continue
            if cmd == 'r':
                s = 120
                send_tank(ser, s, -s)
                continue
            if cmd == 'f':
                s = 120
                send_tank(ser, s, s)
                continue
            if cmd == 'b':
                s = 120
                send_tank(ser, -s, -s)
                continue
            if cmd in ('tank', 'raw') and len(parts) >= 3:
                l = clamp(parts[1])
                r = clamp(parts[2])
                send_tank(ser, l, r)
                continue
            if cmd == 'forward' and len(parts) >= 2:
                s = clamp(parts[1], 0, 255)
                send_tank(ser, s, s)
                continue
            if cmd == 'back' and len(parts) >= 2:
                s = clamp(parts[1], 0, 255)
                send_tank(ser, -s, -s)
                continue
            if cmd == 'left' and len(parts) >= 2:
                s = clamp(parts[1], 0, 255)
                # rotate left in place: left negative, right positive
                send_tank(ser, -s, s)
                continue
            if cmd == 'right' and len(parts) >= 2:
                s = clamp(parts[1], 0, 255)
                # rotate right in place: left positive, right negative
                send_tank(ser, s, -s)
                continue

            # if the user just types two numbers, treat as tank
            if len(parts) == 2:
                # attempt to parse two numbers separated by comma
                if ',' in parts[0]:
                    a = parts[0].split(',')
                    if len(a) == 2:
                        l = clamp(a[0]); r = clamp(a[1]); send_tank(ser, l, r); continue
                # else maybe user typed 'L R'
                try:
                    l = clamp(parts[0]); r = clamp(parts[1]); send_tank(ser, l, r); continue
                except Exception:
                    pass

            print("Unknown command. Type 'help' for usage.")

    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == '__main__':
    main()
