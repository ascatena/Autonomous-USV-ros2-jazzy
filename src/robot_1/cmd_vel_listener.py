import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelListener(Node):
    def __init__(self):
        super().__init__('cmd_vel_listener')

        self.linear_x = 0.0
        self.angular_z = 0.0

        self.subscription = self.create_subscription(
            Twist,
            '/turtle1/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

        self.get_logger().info(
            f'Linear X = {self.linear_x:.2f}, Angular Z = {self.angular_z:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
