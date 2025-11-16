import serial
import struct
import time


class YDLidarX2:
    def __init__(self, port='/dev/serial0', baudrate=128000, motor_pin=23):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.available = False

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)
            self.available = True
            return True
        except serial.SerialException as e:
            print(f"Serial Error: {e}")
            self.available = False
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.available = False

    def start_scan(self):
        if not self.ser or not self.ser.is_open:
            return
        self.ser.write(b'\xA5\x60')  # Start scan command

    def stop(self):
        if not self.ser or not self.ser.is_open:
            return
        self.ser.write(b'\xA5\x65')  # Stop scan command

    def get_data(self):
        if not self.available or not self.ser:
            return [0] * 360

        scan_data = [0] * 360

        try:
            # Wait for a fake scan pattern (simulate)
            for i in range(360):
                scan_data[i] = 1000 + 50 * (i % 10)  # fake values
            return scan_data

        except Exception as e:
            print(f"Error reading data: {e}")
            return scan_data
