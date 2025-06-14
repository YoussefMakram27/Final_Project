#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial
import threading
import time

class AutonomousSTM32MotorNode(Node):
    def __init__(self):
        super().__init__('autonomous_stm32_motor_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('debug_level', 'info')
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.debug_level = self.get_parameter('debug_level').value
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            self.get_logger().info(f"Connected to STM32 on {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to STM32: {e}")
            raise
        
        # Define QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ===========================================
        # SUBSCRIBERS SETUP
        # ===========================================
        
        # Manual control subscriber (Priority 1)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.manual_control_callback,
            qos_profile=qos_profile
        )
        
        # Front ultrasonic sensor subscriber (for emergency brake)
        self.front_ultrasonic_sub = self.create_subscription(
            Range,
            'ultrasonic/front',
            self.front_ultrasonic_callback,
            qos_profile=qos_profile
        )
        
        # ===========================================
        # STATE VARIABLES
        # ===========================================
        
        # Serial communication lock
        self.serial_lock = threading.Lock()
        
        # Manual control state (Priority 1)
        self.manual_mode = False
        self.last_manual_command_time = 0.0
        self.manual_timeout = 2.0  # 2 seconds timeout
        
        # Ultrasonic readings
        self.front_distance = float('inf')
        
        # Emergency brake state (Priority 2)
        self.emergency_brake = False
        self.emergency_brake_start_time = 0.0
        self.emergency_wait_time = 5.0  # 5 seconds wait after obstacle removal
        
        # Speed control
        self.default_speed = 200
        
        # Main control timer
        self.control_timer = self.create_timer(0.1, self.autonomous_control_loop)  # 10Hz
        
        self.get_logger().info("STM32 Motor Node initialized - Manual Control & Emergency Brake only")
    
    # ===========================================
    # CALLBACK FUNCTIONS
    # ===========================================
    
    def manual_control_callback(self, msg):
        """Priority 1: Handle manual control from teleop_keyboard"""
        current_time = time.time()
        
        # DEBUG: Print all incoming messages
        self.get_logger().info(f"[DEBUG] Received: linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}")
        
        # Check if there's actual movement command
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.manual_mode = True
            self.last_manual_command_time = current_time
            
            # Convert twist to command
            linear_x = msg.linear.x
            angular_z = msg.angular.z
            
            if linear_x > 0.0 and abs(angular_z) < 0.1:
                command = "200"  # Forward at speed 200
            elif linear_x < 0.0 and abs(angular_z) < 0.1:
                command = 'B'  # Backward
            elif angular_z > 0.0:
                command = 'L'  # Left turn
            elif angular_z < 0.0:
                command = 'R'  # Right turn
            else:
                command = 'S'  # Stop
            
            self.send_command(command)
            self.get_logger().info(f"[MANUAL MODE] Command: {command}")
        
        else:
            # Handle stop command when no movement is detected
            self.get_logger().info("[DEBUG] Zero movement detected - sending STOP")
            if self.manual_mode:  # Only send stop if we're in manual mode
                self.manual_mode = True  # Keep manual mode active
                self.last_manual_command_time = current_time  # Update timestamp
                self.send_command('S')  # Send stop command
                self.get_logger().info("[MANUAL MODE] Command: S (STOP)")
            else:
                # If not in manual mode, still send stop and enter manual mode
                self.manual_mode = True
                self.last_manual_command_time = current_time
                self.send_command('S')
                self.get_logger().info("[MANUAL MODE] Entering manual mode with STOP")
        
    def front_ultrasonic_callback(self, msg):
        """Update front ultrasonic reading"""
        self.front_distance = msg.range * 100.0  # Convert meters to cm
        if self.debug_level == 'debug':
            self.get_logger().debug(f"Front distance: {self.front_distance:.1f} cm")
    
    # ===========================================
    # MAIN AUTONOMOUS CONTROL LOOP
    # ===========================================
    
    def autonomous_control_loop(self):
        """Main control loop with priority system"""
        current_time = time.time()
        
        # Priority 1: Check if manual mode is active
        if self.manual_mode:
            if (current_time - self.last_manual_command_time) > self.manual_timeout:
                self.manual_mode = False
                self.get_logger().info("[AUTONOMOUS] Returning to autonomous mode")
            else:
                return  # Stay in manual mode
        
        # Priority 2: Emergency brake (Front < 11cm)
        if self.check_emergency_brake():
            return
        
        # Default behavior: Move forward at default speed
        self.move_forward()
    
    # ===========================================
    # PRIORITY CHECK FUNCTIONS
    # ===========================================
    
    def check_emergency_brake(self):
        """Priority 2: Emergency brake for obstacles < 11cm"""
        if self.front_distance < 11.0:
            if not self.emergency_brake:
                self.emergency_brake = True
                self.emergency_brake_start_time = time.time()
                self.get_logger().warn(f"[EMERGENCY BRAKE] Obstacle at {self.front_distance:.1f}cm!")
            
            self.send_command('S')  # Stop
            return True
        else:
            if self.emergency_brake:
                # Check if we should wait before resuming
                wait_time = time.time() - self.emergency_brake_start_time
                if wait_time >= self.emergency_wait_time:
                    self.emergency_brake = False
                    self.get_logger().info("[EMERGENCY BRAKE] Cleared, resuming autonomous mode")
                else:
                    remaining = self.emergency_wait_time - wait_time
                    self.get_logger().info(f"[EMERGENCY BRAKE] Waiting {remaining:.1f}s before resume")
                    self.send_command('S')
                    return True
        
        return False
    
    def move_forward(self):
        """Default behavior: move forward at default speed"""
        self.send_command(f"{self.default_speed:03d}")
        
        if self.debug_level == 'debug':
            self.get_logger().debug(f"[FORWARD] Speed: {self.default_speed}")
    
    # ===========================================
    # UTILITY FUNCTIONS
    # ===========================================
    
    def send_command(self, command):
        """Send command to STM32 with thread safety"""
        try:
            with self.serial_lock:
                self.ser.write(command.encode('utf-8'))
                self.ser.flush()
                
                if self.debug_level == 'debug':
                    self.get_logger().debug(f"Sent: {command}")
                
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
    
    def cleanup(self):
        """Clean up before shutdown"""
        if hasattr(self, 'ser') and self.ser.is_open:
            try:
                with self.serial_lock:
                    self.ser.write(b'S')  # Stop motors
                    self.ser.flush()
                    time.sleep(0.1)
                    self.ser.close()
                    self.get_logger().info("Serial connection closed")
            except Exception as e:
                self.get_logger().error(f"Cleanup error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousSTM32MotorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down autonomous motor node...")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()