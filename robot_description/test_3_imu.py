import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class DisplayIMU(Node):

    def __init__(self):
        # initialize node
        super().__init__("display_imu_node")
        
        # initialize cmd_vel publisher
        self.cmd_vel_pub = self.create_publisher(msg_type=Twist,
                                                 topic="/cmd_vel",
                                                 qos_profile=1)
        # initialize cmd_vel message variable
        self.twist_msg = Twist()

        # initialize scan subscriber
        self.imu_sub_qos = QoSProfile(depth=10,
                                      reliability=ReliabilityPolicy.RELIABLE,
                                      durability=DurabilityPolicy.VOLATILE)
        self.imu_sub = self.create_subscription(msg_type=Imu,
                                                topic="/imu",
                                                callback=self.imu_callback,
                                                qos_profile=self.imu_sub_qos)
        
        # publish twist message with velocities
        self.twist_msg.linear.x = 0.50
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.75
        self.cmd_vel_pub.publish(self.twist_msg)

        return None

    def imu_callback(self, imu_msg):
        # read the values from imu message
        # orientation
        orientation = imu_msg.orientation
        quat_x = orientation.x
        quat_y = orientation.y
        quat_z = orientation.z
        quat_w = orientation.w
        # linear acceleration
        linear_acceleration = imu_msg.linear_acceleration
        linacc_x = linear_acceleration.x
        # angular velocity
        angular_velocity = imu_msg.angular_velocity
        angvel_z = angular_velocity.z

        # print the range value
        self.get_logger().info("qX: %f, qY: %f" % (quat_x, quat_y))
        self.get_logger().info("qZ: %f, qW: %f" % (quat_z, quat_w))
        self.get_logger().info("LinAcc_X: %f" % (linacc_x))
        self.get_logger().info("AngVel_Z: %f" % (angvel_z))
        
        return None

def main(args=None):
    # initialize ROS2 communication
    rclpy.init(args=args)
    # initialize node
    display_imu_node = DisplayIMU()
    # spin the node
    rclpy.spin(display_imu_node)
    # destroy the node
    display_imu_node.destroy_node()
    # shutdown ROS2 communication
    rclpy.shutdown()

    return None

if __name__ == "__main__":
    main()

# End of Code

