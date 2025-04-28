import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from pynput import keyboard

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.manual_pub_ = self.create_publisher(Twist, '/manual_control', 10)
        self.mode_pub_ = self.create_publisher(Bool, '/autonomous_override', 10)
        self.current_twist = Twist()
        self.autonomous_override = False  # Initially in autonomous mode
        self.active_keys = set()

        self.key_mapping = {
            'w': ('linear', 0.2),
            's': ('linear', -0.2),
            'a': ('angular', 0.3),
            'd': ('angular', -0.3),
        }

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()

        self.mode_pub_.publish(Bool(data=self.autonomous_override)) # Publish initial mode

    def publish_manual_command(self):
        """Publishes the current manual velocity command if override is active."""
        if self.autonomous_override:
            self.manual_pub_.publish(self.current_twist)
        else:
            # When in autonomous mode, ensure no manual command is being sent
            stop_twist = Twist()
            self.manual_pub_.publish(stop_twist)

    def on_press(self, key):
        try:
            char = key.char
            if char in self.key_mapping:
                self.active_keys.add(char)
                self.update_twist()
            elif char == ' ':  # Space bar to toggle control
                self.autonomous_override = not self.autonomous_override
                self.mode_pub_.publish(Bool(data=self.autonomous_override))
                self.get_logger().info(f'Autonomous override: {self.autonomous_override}')
                # Immediately publish a zero twist when returning to autonomous
                if not self.autonomous_override:
                    self.current_twist = Twist()
                    self.publish_manual_command()
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            char = key.char
            if char in self.key_mapping and char in self.active_keys:
                self.active_keys.remove(char)
                self.update_twist()
        except AttributeError:
            pass
        if key == keyboard.Key.esc:
            return False

    def update_twist(self):
        """Updates the current twist based on active keys."""
        self.current_twist = Twist()
        for key_char in self.active_keys:
            control_type, value = self.key_mapping[key_char]
            if control_type == 'linear':
                self.current_twist.linear.x += value
            elif control_type == 'angular':
                self.current_twist.angular.z += value
        self.publish_manual_command()

def main(args=None):
    rclpy.init(args=args)
    keyboard_control_node = KeyboardControlNode()
    rclpy.spin(keyboard_control_node)
    keyboard_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()