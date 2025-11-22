import serial
import time

# Connect to Arduino
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

# Connect to camera (OpenMV or similar)
cam = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

time.sleep(2)

last_cam_time = time.time()
cam_timeout = 0.2  # seconds without data before sending default

while True:
    now = time.time()

    if cam.in_waiting:
        data = cam.readline().decode().strip()
        print("cam:", data)

        if "dx:" in data and "dy:" in data:
            try:
                parts = data.split(';')
                dx = int(parts[0].split(':')[1])
                dy = int(parts[1].split(':')[1])

                azimuth = -dx
                elevation = dy
                fire = 1 if abs(dx) < 5 and abs(dy) < 5 else 0

                msg = f"dx:{azimuth};dy:{elevation};F:{fire}\n"
                print("sending:", msg.strip())
                ser.write(msg.encode())

                last_cam_time = now  # Update last time data was received

            except Exception as e:
                print("Error parsing camera data:", e)

    # If no camera data for a while, send zeroed command
    elif now - last_cam_time > cam_timeout:
        azimuth = 0
        elevation = 0
        fire = 0
        msg = f"dx:{azimuth};dy:{elevation};F:{fire}\n"
        print("sending default:", msg.strip())
        ser.write(msg.encode())
        last_cam_time = now  # Prevent spamming

    # Read from Arduino
    if ser.in_waiting:
        print("Arduino:", ser.readline().decode().strip())
