OpenMV as webcam + Pi-side CV
=================================

This folder contains two scripts:

- `openmv_stream.py` — run on the OpenMV H7. Streams JPEG frames over USB VCP
  using a 4-byte little-endian length prefix per frame.

- `pi_person_cv.py` — run on the Raspberry Pi. Reads frames from the OpenMV
  stream, runs a simple OpenCV HOG person detector, and prints `OBJ` lines in
  the format used elsewhere in this repo:

  OBJ <cx> <cy> <dist_m>\n

Usage
-----

1. Flash `openmv_stream.py` to the OpenMV H7 and run it.
2. On the Pi, install dependencies:

   sudo apt update
   sudo apt install -y python3-opencv python3-serial

3. Run the Pi script (edit ports in-script if needed):

   python3 hardware/openmv_as_webcam/pi_person_cv.py

Notes
-----
- `openmv_stream.py` is intentionally simple: it sends raw JPEG frames over
  the serial USB interface. The Pi script decodes them and runs detection.
- Tweak `FRAME_QUALITY` and `FRAME_DELAY_MS` in `openmv_stream.py` to trade
  off FPS vs quality. Lower quality and higher delay -> faster throughput.
- `pi_person_cv.py` uses OpenCV's HOG person detector as a lightweight
  on-device detector; you can replace it with a heavier DNN (e.g. YOLO) on the
  Pi if you need better accuracy.
