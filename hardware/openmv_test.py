"""
openmv_test.py

Minimal OpenMV test script that prints object observations to the USB/serial terminal
in the format expected by the Pi: "OBJ x y d" (or "OBJ -1 -1 -1" when none detected).

Behavior:
- Tries to load a YOLO-LC model at /flash/yolo_lc_192.tflite and run detections if available.
- If the model isn't available, falls back to a simple grayscale blob detector.
- Sends one OBJ line per detection (or -1 line if none), and prints FPS for debugging.

Drop this on the OpenMV board and run from the REPL/serial to see values in your terminal.
"""

import time
import sensor
import image
from pyb import LED

try:
    import ml
    from ml.postprocessing import yolo_lc_postprocess
    HAVE_ML = True
except Exception:
    HAVE_ML = False

# LED indicator
red_led = LED(1)

# Camera setup (matches other scripts: 240x240 window)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.set_windowing((240, 240))
sensor.skip_frames(time=2000)
clock = time.clock()

# Distance estimation params (optional)
KNOWN_WIDTH_M = 0.35
FOCAL_LENGTH_PIX = 300

if HAVE_ML:
    try:
        model = ml.Model("/flash/yolo_lc_192.tflite")
        print("Model loaded:", model)
    except Exception as e:
        print("Model load failed:", e)
        HAVE_ML = False

frame_skip = 0
frame_count = 0

while True:
    clock.tick()
    img = sensor.snapshot()
    frame_count += 1

    if frame_count % (frame_skip + 1) != 0:
        continue

    detected_any = False

    if HAVE_ML:
        # Model.predict returns per-image list of class detections
        boxes = model.predict([img], callback=yolo_lc_postprocess(threshold=0.4))
        for cls_detections in boxes:
            for r, score in cls_detections:
                # r is (x, y, w, h)
                x, y, w, h = r
                cx = int(x + (w / 2))
                cy = int(y + (h / 2))
                dist_m = -1.0
                if w > 0:
                    try:
                        dist_m = (KNOWN_WIDTH_M * FOCAL_LENGTH_PIX) / float(w)
                    except Exception:
                        dist_m = -1.0

                # Send observation to terminal/USB
                print("OBJ %d %d %.3f" % (cx, cy, dist_m))
                detected_any = True

    else:
        # Fallback: simple grayscale blob detection
        GRAYSCALE_THRESHOLD = (0, 80)
        blobs = img.find_blobs([GRAYSCALE_THRESHOLD], area_threshold=150)
        if blobs:
            b = max(blobs, key=lambda x: x.pixels())
            cx = b.cx()
            cy = b.cy()
            w = b.w()
            if w > 0:
                try:
                    dist_m = (KNOWN_WIDTH_M * FOCAL_LENGTH_PIX) / float(w)
                except Exception:
                    dist_m = -1.0
            else:
                dist_m = -1.0

            print("OBJ %d %d %.3f" % (cx, cy, dist_m))
            detected_any = True

    if detected_any:
        red_led.on()
    else:
        red_led.off()
        print("OBJ -1 -1 -1")

    # Helpful debug output
    print("FPS:", clock.fps())

    # small sleep to avoid flooding terminal
    time.sleep_ms(50)
