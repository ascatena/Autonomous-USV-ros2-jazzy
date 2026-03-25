import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

class FixedOdomPublisher(Node):
    def __init__(self):
        super().__init__('fixed_odom_publisher')
        self.pub = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.timer = self.create_timer(0.03, self.timer_callback)  # 30 Hz

    def timer_callback(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"         # frame de referencia
        msg.child_frame_id = "base_link"     # frame del robot

        # covarianza razonable para pruebas (no 0)
        msg.twist.covariance = [
            0.01, 0.0,  0.0,  0.0,   0.0,   0.0,
            0.0,  0.01, 0.0,  0.0,   0.0,   0.0,
            0.0,  0.0,  0.01, 0.0,   0.0,   0.0,
            0.0,  0.0,  0.0,  0.01,  0.0,   0.0,
            0.0,  0.0,  0.0,  0.0,   0.01,  0.0,
            0.0,  0.0,  0.0,  0.0,   0.0,   0.01
        ]

        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FixedOdomPublisher()
    rclpy.spin(node)
