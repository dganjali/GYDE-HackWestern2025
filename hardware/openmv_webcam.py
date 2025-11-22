import sensor, image, time
import pyb
import struct

# Initialize camera
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240 base resolution
sensor.skip_frames(time=2000)

# UART3 on OpenMV, 115200 baud (matches rpi_controller.py)
uart = pyb.UART(3, 115200, timeout_char=1000)

# Simple 4-byte magic header to mark the start of each frame
FRAME_MAGIC = b"FRAM"

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    # Resize to 160x120 to match Pi expectations
    img = img.resize(160, 120)
    # Convert to grayscale to reduce bandwidth
    img = img.to_grayscale()

    # JPEG-compress the frame
    jpeg = img.compress(quality=50)
    data = jpeg.bytearray()
    length = len(data)

    # Send frame as: MAGIC (4 bytes) + length (4 bytes, big-endian) + JPEG data
    try:
        uart.write(FRAME_MAGIC)
        uart.write(struct.pack(">I", length))
        uart.write(data)
    except Exception:
        # Ignore transient UART errors to keep loop running
        pass