import sensor, image, time
import pyb

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # Smaller size for faster transfer
sensor.skip_frames(time = 2000)

uart = pyb.UART(3, 115200)  # Adjust UART port and baudrate

while True:
    img = sensor.snapshot()
    # Optionally crop or resize:
    img = img.resize(160, 120)
    # Convert to grayscale to reduce bandwidth:
    img = img.to_grayscale()
    # Send over UART
    uart.write(img.compress(quality=50).bytearray())

