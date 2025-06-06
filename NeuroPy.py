import serial
import time
import sys
from threading import Thread
import binascii

# Byte codes
CONNECT = b'\xc0'
DISCONNECT = b'\xc1'
AUTOCONNECT = b'\xc2'
SYNC = b'\xaa'
EXCODE = b'\x55'
POOR_SIGNAL = b'\x02'
ATTENTION = b'\x04'
MEDITATION = b'\x05'
BLINK = b'\x16'
HEADSET_CONNECTED = b'\xd0'
HEADSET_NOT_FOUND = b'\xd1'
HEADSET_DISCONNECTED = b'\xd2'
REQUEST_DENIED = b'\xd3'
STANDBY_SCAN = b'\xd4'
RAW_VALUE = b'\x80'

class NeuroPy:
    def __init__(self, port="COM6", baudRate=57600):
        self.port = port
        self.baudRate = baudRate
        self.serial = None
        self.running = False
        self.thread = None

        self.attention = 0
        self.meditation = 0
        self.rawValue = 0
        self.poorSignal = 0
        self.blinkStrength = 0

    def connect(self):
        try:
            self.serial = serial.Serial(self.port, self.baudRate)
            self.serial.flushInput()
            print(f"Connected to {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            return False

    def start(self):
        if not self.serial:
            if not self.connect():
                return
        self.running = True
        self.thread = Thread(target=self._read_loop)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        if self.serial:
            self.serial.close()

    def _read_loop(self):
        while self.running:
            p1 = binascii.hexlify(self.serial.read(1)).decode()
            p2 = binascii.hexlify(self.serial.read(1)).decode()
            if p1 != 'aa' or p2 != 'aa':
                continue

            payload_length = int(binascii.hexlify(self.serial.read(1)).decode(), 16)
            payload = [binascii.hexlify(self.serial.read(1)).decode() for _ in range(payload_length)]
            checksum = int(binascii.hexlify(self.serial.read(1)).decode(), 16)

            # Validate checksum
            calc_checksum = (~sum(int(b, 16) for b in payload)) & 0xFF
            if checksum != calc_checksum:
                continue

            i = 0
            while i < len(payload):
                code = payload[i]
                if code == '02':
                    i += 1
                    self.poorSignal = int(payload[i], 16)
                elif code == '04':
                    i += 1
                    self.attention = int(payload[i], 16)
                elif code == '05':
                    i += 1
                    self.meditation = int(payload[i], 16)
                elif code == '16':
                    i += 1
                    self.blinkStrength = int(payload[i], 16)
                elif code == '80':
                    i += 2
                    val0 = int(payload[i], 16)
                    i += 1
                    val1 = int(payload[i], 16)
                    self.rawValue = val0 * 256 + val1
                    if self.rawValue > 32768:
                        self.rawValue -= 65536
                i += 1

            self._print_values()

    def _print_values(self):
        print(f"Signal: {self.poorSignal} | Attention: {self.attention} | Meditation: {self.meditation} | Raw: {self.rawValue} | Blink: {self.blinkStrength}")

if __name__ == "__main__":
    neuro = NeuroPy(port="COM6")  # Change this to your correct COM port
    try:
        neuro.start()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        neuro.stop()
        print("Disconnected.")
