import serial
import serial.tools.list_ports
import threading
import time

# Default Micro:bit USB VID/PID (may need adjustment for some boards)
MICROBIT_VID = 0x0D28
MICROBIT_PID = 0x0204
BAUDRATE = 115200
TIMEOUT = 1.0

class MicrobitSensor:
    def __init__(self, port=None, baudrate=BAUDRATE):
        self.port = port or self.find_microbit_port()
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.latest_accel = (0, 0, 0)
        self.thread = None
        if self.port:
            self.connect()
        else:
            print("Micro:bit not found. Please check connection.")

    def find_microbit_port(self):
        """Auto-detect the Micro:bit serial port by VID/PID."""
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if (p.vid == MICROBIT_VID and p.pid == MICROBIT_PID):
                return p.device
        return None

    def connect(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=TIMEOUT)
            self.running = True
            self.thread = threading.Thread(target=self._reader, daemon=True)
            self.thread.start()
            print(f"Connected to Micro:bit on {self.port}")
        except Exception as e:
            print(f"Failed to connect to Micro:bit: {e}")
            self.serial = None

    def _reader(self):
        while self.running and self.serial:
            try:
                line = self.serial.readline().decode('utf-8').strip()
                if line:
                    # Expecting format: x, y, z: <x> <y> <z>
                    if line.startswith("x, y, z:"):
                        parts = line.split(":")[-1].strip().split()
                        if len(parts) == 3:
                            try:
                                x, y, z = map(int, parts)
                                self.latest_accel = (x, y, z)
                            except ValueError:
                                pass
            except Exception:
                pass
            time.sleep(0.01)

    def get_accelerometer(self):
        """Return the latest (x, y, z) accelerometer values."""
        return self.latest_accel

    def close(self):
        self.running = False
        if self.serial:
            self.serial.close()
            self.serial = None

if __name__ == "__main__":
    print("Testing MicrobitSensor SDK...")
    mbit = MicrobitSensor()
    try:
        for _ in range(20):
            x, y, z = mbit.get_accelerometer()
            print(f"Accel: x={x}, y={y}, z={z}")
            time.sleep(0.2)
    finally:
        mbit.close()
        print("Closed connection.")
