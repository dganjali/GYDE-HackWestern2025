#!/usr/bin/env python3
"""
pi_tflite_person.py

TFLite-based Raspberry Pi consumer for the OpenMV JPEG stream.

Features:
- Reads length-prefixed JPEG frames from the OpenMV streamer (same format as
  `openmv_stream.py`).
- Runs a TFLite SSD MobileNet-style detector and emits OBJ lines:
    OBJ <cx> <cy> <dist_m>
- Optionally forwards motor commands to an Arduino (same P-only mapping used
  elsewhere in the repo).
- Optional EdgeTPU delegate support (`--edgetpu`).

Usage example:
  python3 hardware/openmv_as_webcam/pi_tflite_person.py --model mobilenet_ssd_v2_320_quant.tflite --labels coco_labels.txt --display --send-cmds

Notes:
- Install runtime: `pip3 install tflite-runtime` (preferred) or `pip3 install tensorflow`.
- If using Coral EdgeTPU, install the EdgeTPU runtime and use `--edgetpu` with an EdgeTPU-compiled model.
"""

import argparse
import sys
import time
import cv2
import numpy as np
import serial
import os

try:
    # prefer lightweight runtime
    from tflite_runtime.interpreter import Interpreter, load_delegate
    TFLITE_RUNTIME = True
except Exception:
    try:
        # fallback to TensorFlow's interpreter
        from tensorflow.lite.python.interpreter import Interpreter
        # load_delegate may be in tensorflow.lite.experimental
        try:
            from tensorflow.lite.python.interpreter import load_delegate
        except Exception:
            try:
                from tensorflow.lite import load_delegate
            except Exception:
                load_delegate = None
        TFLITE_RUNTIME = False
    except Exception:
        print('Error: tflite runtime not available. Install tflite-runtime or tensorflow.', file=sys.stderr)
        sys.exit(1)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--openmv-port', default='/dev/ttyACM0')
    p.add_argument('--openmv-baud', type=int, default=115200)
    p.add_argument('--arduino-port', default=None, help='Serial port to forward motor commands (e.g. /dev/ttyUSB0)')
    p.add_argument('--arduino-baud', type=int, default=115200)
    p.add_argument('--model', required=True, help='Path to TFLite model')
    p.add_argument('--labels', default=None, help='Path to labels file (one label per line)')
    p.add_argument('--threshold', type=float, default=0.4, help='Detection confidence threshold')
    p.add_argument('--display', action='store_true', help='Show preview window')
    p.add_argument('--send-cmds', action='store_true', help='Send motor commands to Arduino')
    p.add_argument('--edgetpu', action='store_true', help='Use EdgeTPU delegate (requires EdgeTPU-compiled model)')
    p.add_argument('--img-w', type=int, default=320)
    p.add_argument('--img-h', type=int, default=320)
    p.add_argument('--cam-fov', type=float, default=60.0)
    p.add_argument('--kp', type=float, default=1.0)
    p.add_argument('--turn-scale', type=float, default=3.5)
    p.add_argument('--max-speed', type=int, default=255)
    p.add_argument('--deadzone', type=float, default=2.0)
    return p.parse_args()


def read_exact(ser, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def load_labels(path):
    if path is None:
        return None
    labels = []
    try:
        with open(path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line:
                    labels.append(line)
    except Exception:
        return None
    return labels


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


def make_interpreter(model_path, use_edgetpu=False):
    delegates = None
    if use_edgetpu:
        # Try to load EdgeTPU delegate
        try:
            if TFLITE_RUNTIME:
                ed = load_delegate('libedgetpu.so.1')
            else:
                ed = load_delegate('libedgetpu.so.1')
            delegates = [ed]
            print('[tflite] EdgeTPU delegate loaded')
        except Exception as e:
            print('[tflite] Warning: failed to load EdgeTPU delegate:', e, file=sys.stderr)
            delegates = None

    if delegates:
        return Interpreter(model_path, experimental_delegates=delegates)
    else:
        return Interpreter(model_path)


def main():
    args = parse_args()

    # Open OpenMV serial
    try:
        openmv = serial.Serial(args.openmv_port, args.openmv_baud, timeout=1)
    except Exception as e:
        print('Failed to open OpenMV port', args.openmv_port, e, file=sys.stderr)
        sys.exit(1)

    arduino = None
    if args.arduino_port:
        try:
            arduino = serial.Serial(args.arduino_port, args.arduino_baud, timeout=1)
        except Exception as e:
            print('Warning: failed to open Arduino port', args.arduino_port, e, file=sys.stderr)
            arduino = None

    labels = load_labels(args.labels)
    person_label_indexes = set()
    if labels:
        for i, l in enumerate(labels):
            if l.strip().lower() == 'person' or l.strip().lower().startswith('person'):
                person_label_indexes.add(i)

    # Load TFLite model
    if not os.path.exists(args.model):
        print('Model not found:', args.model, file=sys.stderr)
        sys.exit(1)

    interp = make_interpreter(args.model, use_edgetpu=args.edgetpu)
    interp.allocate_tensors()

    input_details = interp.get_input_details()
    output_details = interp.get_output_details()
    # assume single input
    in_shape = input_details[0]['shape']
    in_w = in_shape[2]
    in_h = in_shape[1]
    dtype = input_details[0]['dtype']

    print(f'[tflite] model={args.model} input={in_w}x{in_h} dtype={dtype} threshold={args.threshold}')

    print('Ready: reading length-prefixed JPEG frames from', args.openmv_port)

    frames = 0
    t0 = time.time()

    try:
        while True:
            hdr = read_exact(openmv, 4)
            if hdr is None:
                continue
            length = int.from_bytes(hdr, 'little')
            if length <= 0 or length > 500000:
                continue
            data = read_exact(openmv, length)
            if data is None:
                continue

            arr = np.frombuffer(data, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                continue
            frames += 1

            # prepare input
            # convert BGR->RGB
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            resized = cv2.resize(img_rgb, (in_w, in_h))

            if dtype == np.uint8:
                inp = np.expand_dims(resized.astype(np.uint8), axis=0)
            else:
                inp = np.expand_dims(resized.astype(np.float32) / 255.0, axis=0)

            interp.set_tensor(input_details[0]['index'], inp)
            interp.invoke()

            # Typical TFLite SSD outputs: boxes, classes, scores, num
            # But indices vary; we'll search by tensor name if possible.
            boxes = None
            classes = None
            scores = None
            for out in output_details:
                name = out.get('name', '').lower()
                if 'box' in name or 'bbox' in name:
                    boxes = interp.get_tensor(out['index'])
                elif 'class' in name:
                    classes = interp.get_tensor(out['index'])
                elif 'score' in name or 'scores' in name:
                    scores = interp.get_tensor(out['index'])

            # Fallback by order
            if boxes is None and len(output_details) >= 1:
                boxes = interp.get_tensor(output_details[0]['index'])
            if classes is None and len(output_details) >= 2:
                classes = interp.get_tensor(output_details[1]['index'])
            if scores is None and len(output_details) >= 3:
                scores = interp.get_tensor(output_details[2]['index'])

            if boxes is None or classes is None or scores is None:
                # can't parse outputs reliably
                print('[tflite] unable to parse model outputs', file=sys.stderr)
                continue

            # Normalize shapes: [1,num,4], [1,num], [1,num]
            boxes = np.squeeze(boxes)
            classes = np.squeeze(classes)
            scores = np.squeeze(scores)

            h, w = img.shape[:2]

            best = None
            best_area = 0
            for i in range(min(len(scores), 100)):
                sc = float(scores[i])
                if sc < args.threshold:
                    continue
                cls = int(classes[i])
                # If labels provided, prefer matching label
                is_person = False
                if labels and cls < len(labels):
                    if labels[cls].strip().lower().startswith('person'):
                        is_person = True
                else:
                    # assume common models map person to 0 or 1
                    if cls in (0, 1):
                        is_person = True

                if not is_person:
                    continue

                # box format: [ymin, xmin, ymax, xmax] normalized
                by = int(boxes[i][0] * h)
                bx = int(boxes[i][1] * w)
                by2 = int(boxes[i][2] * h)
                bx2 = int(boxes[i][3] * w)
                bw = max(0, bx2 - bx)
                bh = max(0, by2 - by)
                area = bw * bh
                if area > best_area:
                    best_area = area
                    best = (bx, by, bw, bh, sc)

            if best is not None:
                bx, by, bw, bh, sc = best
                cx = int(bx + bw / 2)
                cy = int(by + bh / 2)
                dist_m = -1.0
                if bw > 0:
                    KNOWN_WIDTH_M = 0.35
                    FOCAL_PIX = 300.0
                    dist_m = (KNOWN_WIDTH_M * FOCAL_PIX) / float(bw)
                msg = f"OBJ {cx} {cy} {dist_m:.3f}\n"
            else:
                msg = "OBJ -1 -1 -1\n"

            # output
            print(msg.strip())
            sys.stdout.flush()

            # motor command
            if args.send_cmds and arduino:
                try:
                    if best is None:
                        arduino.write(b"T0,0\n")
                    else:
                        offset_pixels = cx - (args.img_w / 2.0)
                        angle = (offset_pixels / (args.img_w / 2.0)) * (args.cam_fov / 2.0)
                        if abs(angle) <= args.deadzone:
                            left = 0
                            right = 0
                        else:
                            err = -angle
                            turn_output = args.kp * err
                            diff = int(turn_output * args.turn_scale)
                            diff = clamp(diff, -args.max_speed, args.max_speed)
                            left = diff
                            right = -diff
                        cmd = f"T{left:+d},{right:+d}\n"
                        arduino.write(cmd.encode('utf-8'))
                except Exception:
                    pass

            if args.display:
                vis = img.copy()
                if best is not None:
                    cv2.rectangle(vis, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
                    cv2.putText(vis, f"{int(dist_m)}m", (bx, by - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                cv2.imshow('pi_tflite_person', vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # occasional FPS print
            if frames % 60 == 0:
                now = time.time()
                fps = frames / (now - t0 + 1e-9)
                print(f'Processed frames: {frames} fps~{fps:.1f}', file=sys.stderr)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            openmv.close()
        except:
            pass
        if arduino:
            try:
                arduino.close()
            except:
                pass


if __name__ == '__main__':
    main()
