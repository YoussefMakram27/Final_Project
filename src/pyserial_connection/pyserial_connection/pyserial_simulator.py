#!/usr/bin/env python3

import serial
import time
import random
import threading

class SerialSimulator:
    def __init__(self, port='/dev/pts/5', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = True
        self.serial_lock = threading.Lock()
    
    def start(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Simulating sensor data on {self.port}")
            
            # Start threads for sending sensor data and receiving motor commands
            self.send_thread = threading.Thread(target=self.send_sensor_data)
            self.receive_thread = threading.Thread(target=self.receive_motor_commands)
            self.send_thread.start()
            self.receive_thread.start()
            
        except serial.SerialException as e:
            print(f"Serial error: {e}")
    
    def send_sensor_data(self):
        while self.running:
            with self.serial_lock:
                try:
                    # Generate fake sensor data
                    ultrasonic_left = random.uniform(0.02, 4.0)
                    ultrasonic_right = random.uniform(0.02, 4.0)
                    ultrasonic_front = random.uniform(0.02, 4.0)
                    imu_x = random.uniform(-10.0, 10.0)
                    imu_y = random.uniform(-10.0, 10.0)
                    imu_z = random.uniform(-10.0, 10.0)
                    bme_temp = random.uniform(20.0, 30.0)
                    air_quality = random.choice(['F', 'C'])
                    
                    # Format and send data
                    data = f"{ultrasonic_left:.2f},{ultrasonic_right:.2f},{ultrasonic_front:.2f},{imu_x:.2f},{imu_y:.2f},{imu_z:.2f},{bme_temp:.2f},{air_quality}\n"
                    self.serial.write(data.encode('utf-8'))
                    self.serial.flush()
                    print(f"Sent: {data.strip()}")
                    
                except serial.SerialException as e:
                    print(f"Serial error: {e}")
                    break
            time.sleep(0.1)  # Simulate sensor update rate
    
    def receive_motor_commands(self):
        while self.running:
            with self.serial_lock:
                try:
                    # Read motor command
                    line = self.serial.readline().decode('utf-8').strip()
                    if line:
                        print(f"Received motor command: {line}")
                except serial.SerialException as e:
                    print(f"Serial error: {e}")
                    break
                except UnicodeDecodeError:
                    print("Failed to decode motor command")
            time.sleep(0.01)  # Avoid busy-waiting
    
    def stop(self):
        self.running = False
        if self.send_thread.is_alive():
            self.send_thread.join()
        if self.receive_thread.is_alive():
            self.receive_thread.join()
        with self.serial_lock:
            if self.serial and self.serial.is_open:
                self.serial.close()

if __name__ == "__main__":
    simulator = SerialSimulator(port='/dev/pts/5', baudrate=115200)
    try:
        simulator.start()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping simulator")
        simulator.stop()