import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Teleop Node has been started.')
        self.base_linear_speed = 0.5
        self.base_angular_speed = 0.75
        self.base_z_speed = 0.5
        self.increment = 0.1

    def joy_callback(self, msg):
        # Update linear speed
        if msg.buttons[0]:  # Increase linear speed
            self.base_linear_speed += self.increment
        if msg.buttons[3]:  # Decrease linear speed
            self.base_linear_speed = max(0, self.base_linear_speed - self.increment)

        # Update angular speed
        if msg.buttons[1]:  # Increase angular speed
            self.base_angular_speed += self.increment
        if msg.buttons[2]:  # Decrease angular speed
            self.base_angular_speed = max(0, self.base_angular_speed - self.increment)

        # Update z-axis speed
        if msg.buttons[4]:  # Increase z speed (mapped to 't')
            self.base_z_speed += self.increment
        if msg.buttons[5]:  # Decrease z speed (mapped to 'b')
            self.base_z_speed = max(0, self.base_z_speed - self.increment)
                    
        twist = Twist()
        twist.linear.x = self.base_linear_speed * msg.axes[1]  # left stick vertical axis controls forward/backward
        twist.angular.z = self.base_angular_speed * msg.axes[0]  # left stick horizontal axis controls rotation
        twist.linear.z = self.base_z_speed * (msg.buttons[4] - msg.buttons[5])  # z-axis control
        self.get_logger().info('Linear: %f, Angular: %f' % (twist.linear.x, twist.angular.z))
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()