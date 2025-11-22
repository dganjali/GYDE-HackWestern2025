import cv2
import numpy as np
import time
import os
import argparse

# Paths to model files (resolve relative to this script's folder)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(BASE_DIR, "frozen_inference_graph.pb")
CONFIG_PATH = os.path.join(BASE_DIR, "ssd_mobilenet_v2_coco.pbtxt")

def open_camera(index: int | None = None, backend: int | None = None, width: int = 640, height: int = 480):
    """Try to open a camera with sensible fallbacks on macOS."""
    candidates = []
    # If explicit args provided, try only that
    if index is not None:
        if backend is not None:
            candidates.append((index, backend))
        else:
            # Try AVFoundation then any
            candidates.append((index, cv2.CAP_AVFOUNDATION))
            candidates.append((index, cv2.CAP_ANY))
    else:
        # Try common indices with preferred backends
        for i in [0, 1, 2]:
            candidates.append((i, cv2.CAP_AVFOUNDATION))
        for i in [0, 1, 2]:
            candidates.append((i, cv2.CAP_ANY))

    last_err = None
    for idx, be in candidates:
        try:
            cap = cv2.VideoCapture(idx, be)
            if not cap or not cap.isOpened():
                # Some OpenCV builds ignore backend; also try without specifying
                cap = cv2.VideoCapture(idx)
            if cap and cap.isOpened():
                # Try to set modest resolution for stability
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                print(f"Camera opened: index={idx}, backend={be}")
                return cap
            else:
                last_err = f"Could not open camera index {idx} (backend {be})"
        except Exception as e:
            last_err = str(e)
            continue
    print("Failed to open any camera. Last error:", last_err)
    return None


def main():
    parser = argparse.ArgumentParser(description="MobileNetV2 SSD webcam test")
    parser.add_argument("--camera-index", type=int, default=None, help="Camera index (e.g., 0, 1). If omitted, tries common indices.")
    parser.add_argument("--backend", type=str, default="auto", choices=["auto", "avf", "any"], help="Preferred backend: auto, avf (AVFoundation), any")
    parser.add_argument("--width", type=int, default=640, help="Capture width")
    parser.add_argument("--height", type=int, default=480, help="Capture height")
    args = parser.parse_args()

    backend_map = {
        "auto": None,
        "avf": cv2.CAP_AVFOUNDATION,
        "any": cv2.CAP_ANY,
    }

    print("Loading MobileNetV2 SSD model...")
    try:
        # TensorFlow SSD graph + pbtxt
        net = cv2.dnn.readNetFromTensorflow(MODEL_PATH, CONFIG_PATH)
        print("Model loaded successfully.")
    except Exception as e:
        print(f"Error loading model: {e}")
        return

    # Open webcam with fallbacks
    cap = open_camera(index=args.camera_index, backend=backend_map.get(args.backend), width=args.width, height=args.height)
    if cap is None:
        print("Could not open webcam. On macOS, ensure Terminal/VS Code has Camera access in System Settings > Privacy & Security > Camera, and close any app using the camera (Zoom/Teams/FaceTime/Browsers). Try --camera-index 1.")
        return

    print("Starting video stream. Press 'q' to exit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        h, w = frame.shape[:2]

        # MobileNet SSD expects 300x300 input
        blob = cv2.dnn.blobFromImage(frame, size=(300, 300), swapRB=True, crop=False)
        net.setInput(blob)
        
        start = time.time()
        detections = net.forward()
        end = time.time()
        fps = 1 / (end - start)

        # Parse detections - focus on a single best person (largest area among confident detections)
        # Output shape: (1, 1, N, 7) -> [batch, class, score, left, top, right, bottom]
        best_conf = 0.0
        best_box = None
        best_area = 0
        CONF_THRESH = 0.5
        for i in range(detections.shape[2]):
            confidence = float(detections[0, 0, i, 2])
            if confidence >= CONF_THRESH:
                class_id = int(detections[0, 0, i, 1])
                if class_id == 1:  # person
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (sx, sy, ex, ey) = box.astype("int")
                    area = max(0, ex - sx) * max(0, ey - sy)
                    # Prefer largest area; if tie, prefer higher confidence
                    if area > best_area or (area == best_area and confidence > best_conf):
                        best_area = area
                        best_conf = confidence
                        best_box = (sx, sy, ex, ey)

        if best_box is not None:
            (startX, startY, endX, endY) = best_box
            # Center point
            cx = int((startX + endX) / 2)
            cy = int((startY + endY) / 2)
            # Draw only the best one
            label = f"Person: {best_conf:.2f} A:{best_area}"
            cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
            y_text = startY - 15 if startY - 15 > 15 else startY + 15
            cv2.putText(frame, label, (startX, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

        # Display FPS
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("MobileNetV2 SSD Test", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
