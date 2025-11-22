import time
import sensor
import ml
from ml.postprocessing import yolo_lc_postprocess
from pyb import LED, USB_VCP

# Initialize LED
red_led = LED(1)

# Camera setup
sensor.reset()
# Quick experiment toggles - change these to try different trade-offs
# - LOW_QUALITY: lower sensor resolution (faster capture, less detail)
# - WIDE_FRAME: use a wide, short crop (wider horizontal FOV)
# - GRAYSCALE: switch to grayscale capture (only use if your model supports it)
LOW_QUALITY = True
WIDE_FRAME = True
GRAYSCALE = True

if GRAYSCALE:
    sensor.set_pixformat(sensor.GRAYSCALE)
else:
    sensor.set_pixformat(sensor.RGB565)

# Choose framesize. QVGA (320x240) is a reasonable default. QQVGA is much cheaper.
if LOW_QUALITY:
    sensor.set_framesize(sensor.QQVGA)  # 160x120
else:
    sensor.set_framesize(sensor.QVGA)   # 320x240

# Windowing: by default we use a square crop that matches (roughly) the model.
# If you enable WIDE_FRAME we use a wide, short crop (fits within QVGA: 320x160).
# Note: the model input is square; OpenMV will resize/crop as needed before inference.
if WIDE_FRAME and not LOW_QUALITY:
    sensor.set_windowing((320, 160))
elif WIDE_FRAME and LOW_QUALITY:
    # Lower-res wide window (fits within QQVGA 160x120) - use max available width
    sensor.set_windowing((160, 96))
else:
    sensor.set_windowing((240, 240))         # square crop

sensor.skip_frames(time=2000)
# Disable automatic image adjustments to reduce per-frame overhead and improve FPS
try:
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
except Exception:
    # Older firmware may not support toggling; ignore if unavailable
    pass
clock = time.clock()

# Load YOLO LC model from flash
model = ml.Model("/flash/yolo_lc_192.tflite")
model_class_labels = ["person"]
model_class_colors = [(0, 0, 255)]
print("Model loaded:", model)

# Frame skipping to increase FPS (process every (frame_skip+1)th frame)
# Lower this to process more frames. Use 0 to process every frame.
frame_skip = 0
frame_count = 0

# Debug toggle: when True, draw boxes and print FPS. Leave False for max speed.
DEBUG = False

# Distance estimation params (optional, used if we can estimate from bbox width)
KNOWN_WIDTH_M = 0.35   # meters, update for your target
FOCAL_LENGTH_PIX = 300 # tune/calibrate for your lens

usb = USB_VCP()

cb = yolo_lc_postprocess(threshold=0.4)

while True:
    clock.tick()
    img = sensor.snapshot()
    frame_count += 1

    if frame_count % (frame_skip + 1) == 0:
        # Run prediction (reuse callback to avoid recreating closure each frame)
        boxes = model.predict([img], callback=cb)

        # Choose the best detection (highest score) across all classes, and send a single OBJ line.
        best = None
        best_score = 0.0
        for cls_detections in boxes:
            for r, score in cls_detections:
                if score > best_score:
                    best_score = score
                    best = (r, score)

        if best is not None:
            r, score = best
            x, y, w, h = r
            cx = int(x + (w / 2))
            cy = int(y + (h / 2))

            # estimate distance from bbox width if possible
            dist_m = -1.0
            if w > 0:
                try:
                    dist_m = (KNOWN_WIDTH_M * FOCAL_LENGTH_PIX) / float(w)
                except Exception:
                    dist_m = -1.0

            # Draw for debugging (skip in normal mode to save CPU)
            if DEBUG:
                img.draw_rectangle(r)
                img.draw_string(x, max(0, y - 10), "Person", color=(255, 0, 0))

            msg = "OBJ %d %d %.3f\n" % (cx, cy, dist_m)
            try:
                if usb.isconnected():
                    # USB_VCP.write prefers bytes; encode for safety
                    try:
                        usb.write(msg.encode('utf-8'))
                    except Exception:
                        # some firmware versions accept str directly
                        usb.write(msg)
                else:
                    # fallback to REPL print so data is visible in serial
                    print(msg.strip())
            except Exception:
                # swallow write errors to avoid crashing the loop
                pass
            red_led.on()
        else:
            red_led.off()
            # send a single "no object" line
            msg = "OBJ -1 -1 -1\n"
            try:
                if usb.isconnected():
                    try:
                        usb.write(msg.encode('utf-8'))
                    except Exception:
                        usb.write(msg)
                else:
                    print(msg.strip())
            except Exception:
                pass

    # optional FPS print to console (only when debugging)
    if DEBUG:
        print("FPS:", clock.fps())
