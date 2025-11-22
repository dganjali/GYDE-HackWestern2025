# OpenMV script: red_blob_tracker.py
# Finds the largest red blob and sends its coordinates over USB VCP.
# Format: "OBJ <seq> <cx> <cy> <area> <ts_ms>\n"
# If no object is found, it sends: "NOOBJ <seq> <ts_ms>\n"
# This is designed for high-speed operation (15-30+ FPS) at QQVGA.

import sensor, image, time, pyb

# --- Camera Setup ---
sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # Use RGB565, find_blobs is efficient with it
sensor.set_framesize(sensor.QVGA)   # 320x240, wider processing
sensor.skip_frames(time=2000)        # Let sensor stabilize

# --- USB Virtual COM Port ---
# The Pi will see this as a serial device, e.g., /dev/ttyACM0
usb = pyb.USB_VCP()

# --- Red Color Thresholds ---
# These are Lab color space thresholds. Use the OpenMV IDE's Threshold Editor
# to tune these values for your specific lighting and red object.
# Format: (L_min, L_max, a_min, a_max, b_min, b_max)
RED_THRESHOLDS = [
    (30, 100, 15, 127, 15, 127)
]

# --- Main Loop ---
seq = 0
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    # Find blobs matching the red threshold.
    # We merge blobs to treat close patches of red as a single object.
    blobs = img.find_blobs(RED_THRESHOLDS, pixels_threshold=20, area_threshold=100, merge=True)

    ts = time.ticks_ms()
    seq += 1

    if blobs:
        # Find the largest blob by pixel area
        largest_blob = max(blobs, key=lambda b: b.pixels())

        # Get blob properties
        cx = largest_blob.cx()
        cy = largest_blob.cy()
        area = largest_blob.pixels()

        # Draw a box for debugging in the OpenMV IDE framebuffer
        img.draw_rectangle(largest_blob.rect(), color=(0, 255, 0))
        img.draw_cross(cx, cy, color=(0, 255, 0))

        # Send the object data over USB
        if usb.isconnected():
            try:
                line = "OBJ {} {} {} {} {}\n".format(seq, cx, cy, area, ts)
                usb.write(line)
            except OSError:
                # Ignore USB write errors if the host disconnects
                pass
    else:
        # No blob found, send a heartbeat message
        if usb.isconnected():
            try:
                line = "NOOBJ {} {}\n".format(seq, ts)
                usb.write(line)
            except OSError:
                pass

    # Print FPS for debugging in the OpenMV IDE
    # print("FPS:", clock.fps())
