import tkinter as tk
import serial
import threading
import time

# Ports (edit in-file if needed)
ARDUINO_PORT = '/dev/ttyUSB0'
ARDUINO_BAUD = 115200
CAM_PORT = '/dev/ttyACM0'
CAM_BAUD = 115200

CAM_TIMEOUT = 0.2

class SimpleDash:
    def __init__(self, root):
        self.root = root
        root.title('Tank Dashboard')
        self.status = tk.StringVar(value='Waiting for data...')
        self.label = tk.Label(root, textvariable=self.status, font=('Consolas', 12))
        self.label.pack(padx=10, pady=10)

        self.ser_cam = serial.Serial(CAM_PORT, CAM_BAUD, timeout=0.1)
        self.ser_arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.1)

        self.last_cam = time.time()
        self.running = True
        threading.Thread(target=self.read_cam, daemon=True).start()
        threading.Thread(target=self.read_arduino, daemon=True).start()

        root.protocol('WM_DELETE_WINDOW', self.close)

    def read_cam(self):
        while self.running:
            try:
                if self.ser_cam.in_waiting:
                    line = self.ser_cam.readline().decode(errors='ignore').strip()
                    if not line:
                        continue
                    # handle dx;dy;F or OBJ
                    if 'dx:' in line and 'dy:' in line:
                        parts = line.split(';')
                        try:
                            dx = int(parts[0].split(':')[1])
                            dy = int(parts[1].split(':')[1])
                            fire = 1 if abs(dx) < 5 and abs(dy) < 5 else 0
                            msg = f"dx:{-dx};dy:{dy};F:{fire}\n"
                            self.ser_arduino.write(msg.encode())
                            self.status.set(f"dx={dx} dy={dy} fire={fire}")
                            self.last_cam = time.time()
                        except Exception as e:
                            self.status.set('Parse err: ' + str(e))
                    elif line.startswith('OBJ'):
                        # optional: show simple OBJ info
                        self.status.set(line)

                if time.time() - self.last_cam > CAM_TIMEOUT:
                    self.ser_arduino.write(b'dx:0;dy:0;F:0\n')
                    self.status.set('No cam data - sent zeros')
                    self.last_cam = time.time()
                time.sleep(0.01)
            except Exception as e:
                self.status.set('Cam read error: ' + str(e))
                time.sleep(0.1)

    def read_arduino(self):
        while self.running:
            try:
                if self.ser_arduino.in_waiting:
                    line = self.ser_arduino.readline().decode(errors='ignore').strip()
                    if line.startswith('US '):
                        self.status.set('US=' + line[3:])
                    else:
                        print('ARDUINO:', line)
                time.sleep(0.01)
            except Exception as e:
                print('Arduino read error:', e)
                time.sleep(0.1)

    def close(self):
        self.running = False
        try:
            self.ser_cam.close()
            self.ser_arduino.close()
        except:
            pass
        self.root.destroy()

if __name__ == '__main__':
    root = tk.Tk()
    app = SimpleDash(root)
    root.mainloop()
