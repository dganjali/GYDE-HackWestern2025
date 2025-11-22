import tkinter as tk
from PIL import Image, ImageTk
import serial
import threading
import io
import time

# Serial ports
ARDUINO_PORT = '/dev/ttyUSB0'
ARDUINO_BAUD = 9600

CAM_PORT = '/dev/ttyACM0'
CAM_BAUD = 115200

# Timing
CAM_TIMEOUT = 0.2  # seconds to wait before sending default zero command

class TurretGuardApp:
    def __init__(self, root):
        self.root = root
        root.title("TurretGuard Live Feed & Control")
        root.geometry("800x600")
        root.configure(bg="#222")

        # Image label for camera feed
        self.image_label = tk.Label(root, bg="black")
        self.image_label.pack(padx=10, pady=10)

        # Status labels
        self.status_var = tk.StringVar(value="Waiting for data...")
        self.status_label = tk.Label(root, textvariable=self.status_var, font=("Consolas", 14), fg="white", bg="#222")
        self.status_label.pack(pady=5)

        # Open serial ports
        self.ser_arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
        self.ser_cam = serial.Serial(CAM_PORT, CAM_BAUD, timeout=0.1)

        # Last time cam data was received
        self.last_cam_time = time.time()

        # State vars
        self.dx = 0
        self.dy = 0
        self.fire = 0

        # Start threads
        self.running = True
        threading.Thread(target=self.read_camera_data, daemon=True).start()
        threading.Thread(target=self.read_arduino_data, daemon=True).start()
        threading.Thread(target=self.image_feed_loop, daemon=True).start()

        # On close handler
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    def read_camera_data(self):
        """ Continuously read detection data lines from camera serial """
        while self.running:
            try:
                if self.ser_cam.in_waiting:
                    line = self.ser_cam.readline().decode(errors='ignore').strip()
                    if "dx:" in line and "dy:" in line:
                        parts = line.split(';')
                        try:
                            dx = int(parts[0].split(':')[1])
                            dy = int(parts[1].split(':')[1])
                            self.dx = dx
                            self.dy = dy

                            azimuth = -dx
                            elevation = dy
                            fire = 1 if abs(dx) < 5 and abs(dy) < 5 else 0
                            self.fire = fire

                            msg = f"dx:{azimuth};dy:{elevation};F:{fire}\n"
                            self.ser_arduino.write(msg.encode())

                            self.status_var.set(f"dx={dx}, dy={dy}, fire={fire}")

                            self.last_cam_time = time.time()
                        except Exception as e:
                            self.status_var.set(f"Parse error: {e}")

                # If no cam data for timeout period, send zero command
                if time.time() - self.last_cam_time > CAM_TIMEOUT:
                    msg = "dx:0;dy:0;F:0\n"
                    self.ser_arduino.write(msg.encode())
                    self.status_var.set("No camera data - sending zero command")
                    self.last_cam_time = time.time()

                time.sleep(0.01)
            except Exception as e:
                self.status_var.set(f"Camera read error: {e}")
                time.sleep(0.1)

    def read_arduino_data(self):
        """ Continuously read and print Arduino serial data """
        while self.running:
            try:
                if self.ser_arduino.in_waiting:
                    line = self.ser_arduino.readline().decode(errors='ignore').strip()
                    if line:
                        print("Arduino:", line)
            except Exception as e:
                print("Arduino read error:", e)
                time.sleep(0.1)

    def image_feed_loop(self):
        """ Read JPEG images from camera serial and display in UI """
        while self.running:
            try:
                # Look for JPEG start marker
                start = self.ser_cam.read(2)
                if start != b'\xFF\xD8':
                    continue  # Not start of JPEG, skip

                img_data = bytearray(start)
                while True:
                    b = self.ser_cam.read(1)
                    if not b:
                        break
                    img_data.extend(b)
                    if img_data[-2:] == b'\xFF\xD9':  # JPEG end
                        break

                from PIL import Image
                img = Image.open(io.BytesIO(img_data))
                img = img.resize((640, 480))
                img_tk = ImageTk.PhotoImage(img)

                # Update UI in main thread
                self.root.after(0, self.update_image, img_tk)
            except Exception as e:
                # Ignore corrupt images or timeout
                #print("Image error:", e)
                time.sleep(0.01)

    def update_image(self, img_tk):
        self.image_label.configure(image=img_tk)
        self.image_label.image = img_tk

    def on_close(self):
        self.running = False
        try:
            self.ser_cam.close()
            self.ser_arduino.close()
        except:
            pass
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = TurretGuardApp(root)
    root.mainloop()
