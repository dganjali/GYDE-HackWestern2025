# Edge Impulse - OpenMV FOMO Object Detection Algorithm for Drowning Detection
#
# This work is licensed under the MIT license.
# Copyright (c) 2013-2024 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE

## SAME Root Directory As DDS1

##### MODEL STACK | Pipeline:
  # - Edge Impulse Framework for Edge AI Computing
  # - 1615 Image Samples: (96x96 Resolution + "Fit-Shortest" Axis Resizing) |  85% - 15% Train-Test Split
  # - JSON Labelling Matrices for Feature Preprocessing
  # - FOMO (Fast Object Motion Detection) MobileNetV2 0.35 (TinyML Neural Network for Real-time Object Detection)
  # - Deployed Real-time on OpenMV H7 Cam Plus (Computer-Vision based Microcontroller) - Built in MicroPython (Python3 Lite)
  # - 4 Feature Class Segmentations
  # - NN Settings: 60 Epoch Architecture, GPU Training Processor, NN Learning Optimizer Enabled, Data Augmentation, int8 Quantization | Input Layer: 27,648 Features
  # - RAM-Optimized Engine: EON (Edge Optimized Neural) Compiler
  # - .eim OpenMV Firmware + OpenMV Library DFU Integration

## Note how Feauture Class Label Mappings Appear on Edge Impulse



import sensor
import image
import time
import ml, math, uos, gc

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240, 240))  # 240x240
sensor.skip_frames(time=2000)

net = None
labels = None
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
    (255,   0,   0),
    (  0, 255,   0),
    (  0,   0, 255),
    (255,   0, 255),
    (255, 255,   0),
    (  0, 255, 255),
    (255, 255, 255),
]

threshold_list = [(math.ceil(min_confidence * 255), 255)]

def fomo_post_process(model, inputs, outputs):
    ob, oh, ow, oc = model.output_shape[0]

    x_scale = inputs[0].roi[2] / ow
    y_scale = inputs[0].roi[3] / oh
    scale = min(x_scale, y_scale)

    x_offset = ((inputs[0].roi[2] - (ow * scale)) / 2) + inputs[0].roi[0]
    y_offset = ((inputs[0].roi[3] - (ow * scale)) / 2) + inputs[0].roi[1]

    l = [[] for i in range(oc)]

    for i in range(oc):
        img = image.Image(outputs[0][0, :, :, i] * 255)
        blobs = img.find_blobs(threshold_list, x_stride=1, y_stride=1, area_threshold=1, pixels_threshold=1)
        for b in blobs:
            rect = b.rect()
            x, y, w, h = rect
            score = img.get_statistics(thresholds=threshold_list, roi=rect).l_mean() / 255.0
            x = int((x * scale) + x_offset)
            y = int((y * scale) + y_offset)
            w = int(w * scale)
            h = int(h * scale)
            l[i].append((x, y, w, h, score))
    return l

# === Pitch angle calculation ===
def get_pitch_angle(dy, image_height=240, vFOV_deg=60):
    return (dy / (image_height / 2)) * (vFOV_deg / 2)

clock = time.clock()
while(True):
    clock.tick()
    img = sensor.snapshot()
    center_img_x = 240 // 2
    center_img_y = 240 // 2

    overlay_info = ""  # For overlay text

    # Draw center crosshair
    img.draw_line(center_img_x - 4, center_img_y, center_img_x + 4, center_img_y, color=(200, 200, 200))
    img.draw_line(center_img_x, center_img_y - 4, center_img_x, center_img_y + 4, color=(200, 200, 200))

    for i, detection_list in enumerate(net.predict([img], callback=fomo_post_process)):
        if i == 0: continue
        if len(detection_list) == 0: continue

        class_name = labels[i]

        for x, y, w, h, score in detection_list:
            center_x = math.floor(x + (w / 2))
            center_y = math.floor(y + (h / 2))
            dx = center_x - center_img_x
            dy = center_y - center_img_y
            pitch = get_pitch_angle(dy)

            if(score > 0.85):
                is_drowning = "drown" in class_name.lower()

                # Adjust bounding box for non-drowning to upper part (head-level)
                if not is_drowning:
                    new_h = int(h * 0.4)
                    img.draw_rectangle((x, y, w, new_h), color=colors[i], thickness=2)
                    img.draw_circle((center_x, y + new_h // 2, 8), color=colors[i])
                    label_position_y = y - 10
                else:
                    img.draw_rectangle((x, y, w, h), color=colors[i], thickness=2)
                    img.draw_circle((center_x, center_y, 10), color=colors[i])
                    label_position_y = y - 10

                label_text = "Human" if "person" in class_name.lower() else class_name
                img.draw_string(x, label_position_y, label_text, mono_space=False, color=colors[i], scale=1)

                overlay_info += f"{label_text}: dx={dx} dy={dy} pitch={pitch:.1f}° score={score:.2f}\n"

                # Print to serial (human readable)
                print(f"{label_text} | x={center_x}, y={center_y}, dx={dx}, dy={dy}, pitch={pitch:.1f}°, score={score:.3f}")

                # Compact machine-friendly line for Pi parsing: DX (pixels), H (bbox height pixels)
                # Example: "DX:-12;H:48"
                try:
                    print(f"DX:{dx};H:{h}")
                except Exception:
                    # if h not available for some reason, still print DX only
                    print(f"DX:{dx};H:0")

    # Display overlay text at top left
                img.draw_string(2, 2, overlay_info.strip(), scale=1, mono_space=False, color=(255, 255, 255))

    print(clock.fps(), "fps\n")
