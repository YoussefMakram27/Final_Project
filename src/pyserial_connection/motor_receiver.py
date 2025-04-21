import serial
import time

ser = serial.Serial('/dev/pts/3', 115200, timeout=1)

while True:
    received = ser.readline().decode().strip()
    if received:
        print(f"STM32 Received: {received}")  # Simulate STM32 processing command
        ser.write(b"ACK\n")  # Simulate an acknowledgment
    time.sleep(0.1)  # Simulate processing delay