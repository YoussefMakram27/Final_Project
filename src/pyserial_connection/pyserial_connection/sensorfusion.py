#!/usr/bin/env python3
import serial
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import UInt8, String

class SerialDebugger(Node):
    def __init__(self):
        super().__init__('serial_debugger')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)  # Updated to match STM32 baudrate
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # Publishers
        self.data_pub = self.create_publisher(UInt8, 'stm_data', 10)
        self.debug_pub = self.create_publisher(String, 'stm_debug', 10)
        self.status_pub = self.create_publisher(String, 'stm_status', 10)
        
        # Try different serial port settings
        self.get_logger().info("Attempting to open serial port with different settings...")
        
        self.ser = None
        self.open_attempts = 0
        self.max_attempts = 5
        self.current_baudrate = baudrate
        
        # Common baudrates to try - prioritize 115200 as we know STM32 is using this
        self.baudrates = [115200, 9600, 19200, 38400, 57600]
        if baudrate in self.baudrates:
            # Move the specified baudrate to the front
            self.baudrates.remove(baudrate)
            self.baudrates.insert(0, baudrate)
        else:
            self.baudrates.insert(0, baudrate)
        
        # Create a timer that attempts to open the port
        self.open_timer = self.create_timer(1.0, self.try_open_port)
        
        # Buffer for collecting incoming data
        self.buffer = bytearray()
        
        # Track statistics
        self.reset_time = self.get_clock().now()
        self.bytes_received = 0
        self.s_commands = 0  # Count of 'S' commands
        self.f_commands = 0  # Count of 'F' commands
        self.unknown_commands = 0  # Count of unknown commands
        self.stats_timer = self.create_timer(5.0, self.print_stats)
    
    def try_open_port(self):
        """Try to open the serial port with different settings"""
        if self.ser is not None and self.ser.is_open:
            # Port is already open, stop the timer
            self.open_timer.cancel()
            return
            
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.baudrates[self.open_attempts % len(self.baudrates)]
        
        self.get_logger().info(f"Attempt {self.open_attempts+1}: Opening {port} at {baudrate} baud")
        
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.current_baudrate = baudrate
            self.ser.reset_input_buffer()
            self.get_logger().info(f"SUCCESS! Port opened at {baudrate} baud.")
            
            # Stop the open timer
            self.open_timer.cancel()
            
            # Start the read timer
            self.read_timer = self.create_timer(0.05, self.read_serial)
            
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open port: {e}")
            self.open_attempts += 1
            
            if self.open_attempts >= self.max_attempts * len(self.baudrates):
                self.get_logger().error("Maximum open attempts reached. Giving up.")
                self.open_timer.cancel()
                rclpy.shutdown()
    
    def read_serial(self):
        """Read from the serial port with extensive diagnostics"""
        if self.ser is None or not self.ser.is_open:
            return
            
        try:
            # Check for available data
            if self.ser.in_waiting > 0:
                # Read all available bytes
                data = self.ser.read(self.ser.in_waiting)
                self.bytes_received += len(data)
                
                if data:
                    # Publish raw data for debugging
                    debug_msg = String()
                    debug_msg.data = ' '.join([f'{b:02X}' for b in data])
                    self.debug_pub.publish(debug_msg)
                    
                    # Log the raw hex
                    self.get_logger().info(f"Raw data ({len(data)} bytes): {debug_msg.data}")
                    
                    # Try to interpret as text
                    try:
                        text = data.decode('utf-8').strip()
                        if text:
                            self.get_logger().info(f"As text: '{text}'")
                    except UnicodeDecodeError:
                        pass
                    
                    # Add to buffer and process
                    self.buffer.extend(data)
                    self.process_buffer()
        
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error during read: {e}")
            # Try to reopen the port
            if self.ser.is_open:
                self.ser.close()
            self.open_attempts = 0
            self.open_timer = self.create_timer(1.0, self.try_open_port)
            self.read_timer.cancel()
    
    def process_buffer(self):
        """Process the buffer for complete messages"""
        # Look for newline characters in the buffer
        while b'\n' in self.buffer:
            # Split at the first newline
            line_end = self.buffer.find(b'\n')
            line = self.buffer[:line_end].decode('utf-8', errors='replace').strip()
            
            # Remove the processed line from the buffer
            self.buffer = self.buffer[line_end + 1:]
            
            # Process the line
            self.process_command(line)
    
    def process_command(self, command):
        """Process a command from the STM32"""
        # Publish the command
        status_msg = String()
        status_msg.data = command
        self.status_pub.publish(status_msg)
        
        if command == 'S':
            self.s_commands += 1
            self.get_logger().info("✅ Received START command")
            # You can add specific actions for 'S' command here
        elif command == 'F':
            self.f_commands += 1
            self.get_logger().info("✅ Received FINISH command")
            # You can add specific actions for 'F' command here
        else:
            self.unknown_commands += 1
            self.get_logger().warn(f"❓ Unknown command: '{command}'")
        
        # Also publish each byte of the command for compatibility
        for byte in command.encode('utf-8'):
            msg = UInt8()
            msg.data = byte
            self.data_pub.publish(msg)
    
    def print_stats(self):
        """Print statistics about the communication"""
        now = self.get_clock().now()
        elapsed = (now - self.reset_time).nanoseconds / 1e9
        
        self.get_logger().info("\n--- SERIAL STATISTICS ---")
        self.get_logger().info(f"Port: {self.get_parameter('port').get_parameter_value().string_value} at {self.current_baudrate} baud")
        self.get_logger().info(f"Running for: {elapsed:.1f} seconds")
        self.get_logger().info(f"Bytes received: {self.bytes_received}")
        self.get_logger().info(f"Average byte rate: {self.bytes_received/elapsed if elapsed > 0 else 0:.1f} bytes/sec")
        self.get_logger().info(f"START ('S') commands: {self.s_commands}")
        self.get_logger().info(f"FINISH ('F') commands: {self.f_commands}")
        self.get_logger().info(f"Unknown commands: {self.unknown_commands}")
        self.get_logger().info("------------------------\n")

def main(args=None):
    rclpy.init(args=args)
    node = SerialDebugger()
    
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