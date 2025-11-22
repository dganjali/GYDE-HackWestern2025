MVP Software Pipeline (Serial-Based)
===================================

This repo contains code from an earlier project. The files here can be used
as the foundation for a simple 3-layer serial-based robot follower:

1) OpenMV Cam (perception)
2) Raspberry Pi (decision / integration)
3) Arduino Nano (low-level control & safety)

This document summarizes the minimal serial protocol, what's already in the
repo, and how to run the new example components.

Message formats (lightweight, numeric, no JSON)
------------------------------------------------

OpenMV -> Raspberry Pi (existing code prints several formats). Recommended
minimal line that Pi can parse (we try to be permissive):

  (examples found in repo)
  "x 120\ty 110\tdx= -12\tdy= 3\tscore 0.92"
  or
  "x 120\ty 110 dx:-12 dy:3 pitch:2.1 score:0.92"

Important: the Pi controller reads `dx` (horizontal pixel offset) and an
optional bounding-box height `h` (or `w`) if printed. If `h` is present the
Pi will estimate distance using a very simple inverse model; you should
calibrate the constant for your camera.


Raspberry Pi -> Arduino (new stable compact format)
--------------------------------------------------

The Pi sends two numbers in a single ASCII line, labelled and separated by a
semicolon. Example:

  L:0.120;A:-0.450\n
Meaning: linear speed in m/s and angular speed in rad/s. Both are plain numbers
with fixed labels so the Arduino can parse quickly.


Files added in this workspace
----------------------------

- `pi_controller.py` — new: reads OpenMV serial lines, computes a conservative
  linear and angular command, forwards to Arduino as `L:...;A:...`.
- `arduino_sketches/nano_controller/nano_controller.ino` — new: Arduino sketch
  that parses the `L` and `A` values, enforces safety limits, checks an
  E-stop, and mixes speeds to differential motor outputs.


How to try locally (example)
----------------------------

1. Copy/flash the OpenMV script to the OpenMV cam (use the existing
   `OpenMV/*.py` scripts). Ensure OpenMV prints lines that include `dx`.

2. On the Raspberry Pi, run the Pi controller (adjust device paths):

```bash
python3 pi_controller.py --openmv-port /dev/ttyACM0 --arduino-port /dev/ttyUSB0
```

Start with `--dry-run` to only print decisions without sending to Arduino.

3. Upload the Nano sketch in `arduino_sketches/nano_controller/nano_controller.ino`
   to your Arduino Nano. Adjust motor pins and safety pin wiring at the top
   of the sketch to match your hardware.


Notes, calibration & next steps
-------------------------------
- Distance estimation from bounding box height is only a rough heuristic
  (distance = K / h). Calibrate K by measuring known distances and bounding
  box heights.
- Tuning: adjust `MAX_LINEAR`, `MAX_ANGULAR`, and gains in `pi_controller.py`.
- Add obstacle sensing on Pi or forward ultrasonic telemetry from Arduino for
  more advanced avoidance.
- Optionally change the message format to fixed-width binary if you need
  lower-latency/robust comms.

If you'd like, I can:
- wire these pieces together in the repo (update existing OpenMV prints to
  output `dx` & `h` consistently),
- add a small calibration helper (collect h vs distance), or
- add telemetry messages from the Arduino back to the Pi (battery, distances).
