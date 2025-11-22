# TurretGuard Drowning Detection - OpenMV + Edge Impulse FOMO
# MIT License © 2013-2024 OpenMV LLC, adapted for TurretGuard by Daniel Ganjali

import sensor, image, time, ml, math, uos, gc
from pyb import USB_VCP

usb = USB_VCP()

# Camera setup
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)      # 320x240
sensor.set_windowing((240, 240))       # square crop
sensor.skip_frames(time=2000)

# Model and labels
min_confidence = 0.5
try:
    net = ml.Model("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
except Exception as e:
    raise Exception('Failed to load "trained.tflite": ' + str(e))

try:
    labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception as e:
    raise Exception('Failed to load "labels.txt": ' + str(e))

colors = [
    (255,   0,   0), (0, 255,   0), (0, 0, 255),
    (255,   0, 255), (255, 255, 0), (0, 255, 255), (255, 255, 255)
]

threshold_list = [(math.ceil(min_confidence * 255), 255)]

def fomo_post_process(model, inputs, outputs):
    ob, oh, ow, oc = model.output_shape[0]
    x_scale = inputs[0].roi[2] / ow
    y_scale = inputs[0].roi[3] / oh
    scale = min(x_scale, y_scale)
    x_offset = ((inputs[0].roi[2] - (ow * scale)) / 2) + inputs[0].roi[0]
    y_offset = ((inputs[0].roi[3] - (ow * scale)) / 2) + inputs[0].roi[1]

    l = [[] for _ in range(oc)]
    for i in range(oc):
        img_out = image.Image(outputs[0][0, :, :, i] * 255)
        blobs = img_out.find_blobs(threshold_list, x_stride=1, y_stride=1, area_threshold=1, pixels_threshold=1)
        for b in blobs:
            x, y, w, h = b.rect()
            score = img_out.get_statistics(thresholds=threshold_list, roi=b.rect()).l_mean() / 255.0
            x = int((x * scale) + x_offset)
            y = int((y * scale) + y_offset)
            w = int(w * scale)
            h = int(h * scale)
            l[i].append((x, y, w, h, score))
    return l

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    center_img_x = 240 // 2
    center_img_y = 240 // 2
    detected = False
    dx, dy = 0, 0

    for i, detection_list in enumerate(net.predict([img], callback=fomo_post_process)):
        if i == 0 or len(detection_list) == 0:
            continue

        label = labels[i]
        print("********** %s **********" % label)

        # Process first detection of class i
        x, y, w, h, score = detection_list[0]
        center_x = math.floor(x + w / 2)
        center_y = math.floor(y + h / 2)
        dx = center_x - center_img_x
        dy = center_y - center_img_y
        detected = True

        # Draw overlay
        img.draw_rectangle((x, y, w, h), color=colors[i])
        img.draw_circle((center_x, center_y, 12), color=colors[i])
        img.draw_string(x, y - 12, "{:.2f}".format(score), color=colors[i])

        # Print debug info
        print(f"x {center_x}\ty {center_y}\tscore {score:.3f}\tdx {dx}\tdy {dy}")
        break  # Only send one detection per frame

    if not detected:
        dx, dy = 0, 0

    # ---- SERIAL OUTPUT SECTION ----
    try:
        # Send dx/dy status line over USB serial (for Pi control logic)
        status_line = f"dx:{dx};dy:{dy}\n"
        usb.write(status_line)

        # Send compressed JPEG image over USB serial (for Pi live feed)
        b = img.compress(quality=90).bytearray()
        usb.write(b)

    except Exception as e:
        print("Serial error:", e)

    # Print FPS (optional)
    print(clock.fps(), "fps\n")
