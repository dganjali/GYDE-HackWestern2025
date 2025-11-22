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
sensor.set_framesize(sensor.QVGA)      # 320x240
sensor.set_windowing((240, 240))       # square crop
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
    (255,   0,   0),   # red - class 0
    (  0, 255,   0),   # green - class 1
    (  0,   0, 255),   # blue - class 2
    (255,   0, 255),   # magenta - class 3
    (255, 255,   0),   # yellow - class 4
    (  0, 255, 255),   # cyan - class 5
    (255, 255, 255),   # white - class 6
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

clock = time.clock()
while(True):
    clock.tick()
    img = sensor.snapshot()

    center_img_x = 240 // 2
    center_img_y = 240 // 2

    for i, detection_list in enumerate(net.predict([img], callback=fomo_post_process)):
        if i == 0: continue
        if len(detection_list) == 0: continue

        print("********** %s **********" % labels[i])
        for x, y, w, h, score in detection_list:
            center_x = math.floor(x + (w / 2))
            center_y = math.floor(y + (h / 2))
            dx = center_x - center_img_x
            dy = center_y - center_img_y

            # Draw circle at center
            img.draw_circle((center_x, center_y, 12), color=colors[i])

            #  Draw bounding box
            img.draw_rectangle((x, y, w, h), color=colors[i])

            # Print full detection info including dx, dy
            print(f"x {center_x}\ty {center_y}\tscore {score:.3f}\tdx {dx}\tdy {dy}")

    print(clock.fps(), "fps", end="\n\n")
