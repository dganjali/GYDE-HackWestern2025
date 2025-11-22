import sensor, image, time
import pyb
import struct

# Initialize camera
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 320x240 base resolution
sensor.skip_frames(time=2000)

# Use USB Virtual COM Port so frames go over USB (Pi sees /dev/ttyACM*)
usb = pyb.USB_VCP()

# Simple 4-byte magic header to mark the start of each frame
FRAME_MAGIC = b"FRAM"

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    # Convert to grayscale to reduce bandwidth
    img = img.to_grayscale()

    # JPEG-compress the frame
    jpeg = img.compress(quality=50)
    data = jpeg.bytearray()
    length = len(data)

    # Send frame as: MAGIC (4 bytes) + length (4 bytes, big-endian) + JPEG data
    try:
        if usb.isconnected():
            usb.write(FRAME_MAGIC)
            usb.write(struct.pack(">I", length))
            usb.write(data)
        else:
            # USB not connected to host; skip sending this frame
            pass
    except Exception:
        # Ignore transient USB errors to keep loop running
        pass