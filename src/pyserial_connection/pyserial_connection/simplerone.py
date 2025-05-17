#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSerialReader(Node):
    def __init__(self):
        super().__init__('simple_serial_reader')
        
        # Set fixed baudrate to 115200 since we know that's what the STM32 uses
        self.port = '/dev/ttyUSB0'
        self.baudrate = 115200
        
        # Create publisher for commands
        self.command_pub = self.create_publisher(String, 'stm_command', 10)
        
        # Initialize serial port
        self.get_logger().info(f"Opening {self.port} at {self.baudrate} baud")
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.get_logger().info("Serial port opened successfully")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open port: {e}")
            rclpy.shutdown()
            return
            
        # Buffer for collecting bytes until a complete message
        self.buffer = bytearray()
        
        # Create timer to read from serial port
        self.timer = self.create_timer(0.05, self.read_serial)
        
        # Counters for commands
        self.s_count = 0
        self.f_count = 0
    
    def read_serial(self):
        """Read from serial port and process any complete messages"""
        if not self.ser.is_open:
            return
            
        try:
            # Read any available data
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                
                # Add to our buffer
                self.buffer.extend(data)
                
                # Process any complete messages in buffer
                self.process_buffer()
                
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            if self.ser.is_open:
                self.ser.close()
    
    def process_buffer(self):
        """Process the buffer for complete messages ending with newline"""
        # Critical part: Process buffer until we find complete messages
        while b'\n' in self.buffer:
            # Find the end of the first complete message
            newline_pos = self.buffer.find(b'\n')
            
            # Extract the message and convert to string
            message = self.buffer[:newline_pos].decode('utf-8', errors='replace').strip()
            
            # Remove the processed message from buffer
            self.buffer = self.buffer[newline_pos + 1:]
            
            # Handle the command
            self.handle_command(message)
    
    def handle_command(self, command):
        """Process a command from the STM32"""
        # Publish the command
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        
        # Handle specific commands
        if command == 'S':
            self.s_count += 1
            self.get_logger().info(f"{command}")
        elif command == 'F':
            self.f_count += 1
            self.get_logger().info(f"{command}")
        else:
            self.get_logger().info(f"Unknown command: '{command}'")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSerialReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser and node.ser.is_open:
            node.ser.close()
            node.get_logger().info("Serial port closed")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()