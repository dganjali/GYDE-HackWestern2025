# openmv_obj.py
#
# Detect a blob/contour and send: "OBJ x y d\n"
# x,y are pixel coords (center), d is distance in meters (float)
#
# Tune THRESHOLD or use find_blobs with color thresholds depending on target.

import sensor, image, time, pyb

# --- CONFIG ---
# Choose method: 'color' or 'grayscale'
MODE = 'grayscale'

# Grayscale threshold (for grayscale blobs) or color threshold for color mode
GRAYSCALE_THRESHOLD = (0, 80)   # tune
# Example color threshold (L Min, L Max, A Min, A Max, B Min, B Max) — tune if using color:
COLOR_THRESHOLD = (30, 100, -10, 10, -10, 10)

# Known real-world width of the target (meters) and focal length in pixels
KNOWN_WIDTH_M = 0.35   # e.g., shoulder width ~35 cm — update to target
FOCAL_LENGTH_PIX = 300 # calibrate: focal = (pixel_width * known_distance) / known_width

# Minimum blob area to accept (pixels)
MIN_AREA = 150

# Serial / timing
baud = 115200
usb = pyb.USB_VCP()
clock = time.clock()

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # 320x240
sensor.skip_frames(200)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

while True:
    clock.tick()
    img = sensor.snapshot()

    # Choose detection mode
    if MODE == 'grayscale':
        blobs = img.find_blobs([GRAYSCALE_THRESHOLD], area_threshold=MIN_AREA, pixels_threshold=MIN_AREA)
    else:
        blobs = img.find_blobs([COLOR_THRESHOLD], area_threshold=MIN_AREA, pixels_threshold=MIN_AREA)

    if blobs:
        # pick largest blob
        b = max(blobs, key=lambda x: x.pixels())
        cx = b.cx()
        cy = b.cy()
        # estimate width in pixels (use rect or width property)
        pixel_width = b.w() if b.w() > 0 else b.r()

        # Simple distance estimate: distance = (known_width * focal_length) / pixel_width
        if pixel_width > 0:
            dist_m = (KNOWN_WIDTH_M * FOCAL_LENGTH_PIX) / float(pixel_width)
        else:
            dist_m = -1.0

        # Send the observation
        # Format: OBJ x y d\n
        msg = "OBJ %d %d %.3f\n" % (cx, cy, dist_m)
        try:
            usb.write(msg)
        except:
            pass

        # optional drawing for debugging
        img.draw_rectangle(b.rect(), color=(255,0,0))
        img.draw_cross(cx, cy, color=(0,255,0))
    else:
        # No object: optionally notify
        try:
            usb.write("OBJ -1 -1 -1\n")
        except:
            pass
