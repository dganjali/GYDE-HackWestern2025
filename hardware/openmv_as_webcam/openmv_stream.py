import sensor, image, pyb
from pyb import USB_VCP

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

usb = USB_VCP()

while True:
    img = sensor.snapshot()
    buf = img.compress(quality=50).bytearray()  # JPEG compressed to reduce size
    size = len(buf)
    usb.write(size.to_bytes(4, 'little'))       # send size first
    usb.write(buf)                              # send image data
