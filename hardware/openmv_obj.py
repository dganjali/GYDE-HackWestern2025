import time
import sensor
import ml
from ml.postprocessing import yolo_lc_postprocess
from pyb import LED, USB_VCP

# Initialize LED
red_led = LED(1)

# Camera setup
sensor.reset()
sensor.set_pixformat(sensor.RGB565)      # Keep RGB
# Use QVGA to match a 240x240 windowing below (avoid invalid window sizes)
sensor.set_framesize(sensor.QVGA)       # 320x240
sensor.set_windowing((240, 240))         # square crop
sensor.skip_frames(time=2000)
clock = time.clock()

# Load YOLO LC model from flash
model = ml.Model("/flash/yolo_lc_192.tflite")
model_class_labels = ["person"]
model_class_colors = [(0, 0, 255)]
print("Model loaded:", model)

# Frame skipping to increase FPS
frame_skip = 5   # process every (frame_skip+1)th frame
frame_count = 0

# Distance estimation params (optional, used if we can estimate from bbox width)
KNOWN_WIDTH_M = 0.35   # meters, update for your target
FOCAL_LENGTH_PIX = 300 # tune/calibrate for your lens

usb = USB_VCP()

while True:
    clock.tick()
    img = sensor.snapshot()
    frame_count += 1

    if frame_count % (frame_skip + 1) == 0:
        # Run prediction
        boxes = model.predict([img], callback=yolo_lc_postprocess(threshold=0.4))

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

            # Draw for debugging
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

    # optional FPS print to console
    print("FPS:", clock.fps())
