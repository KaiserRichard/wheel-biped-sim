"""AGV-style /cmd_vel rehearsal; main is the only ROS import boundary."""

from .core import DEFAULT_PORT, Forwarder


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Empty

    class Bridge(Node):
        def __init__(self):
            super().__init__('twin_cmd_bridge')
            self.declare_parameter('host', 'localhost')
            self.declare_parameter('port', DEFAULT_PORT)
            self.declare_parameter('src', '')
            self.forwarder = Forwarder(self.get_parameter('host').value,
                                       self.get_parameter('port').value,
                                       self.get_parameter('src').value or None)
            # ASSUMED queue depth 1 prevents an old command backlog.
            self.velocity_sub = self.create_subscription(Twist, '/cmd_vel', self.on_twist, 1)
            self.jump_sub = self.create_subscription(Empty, '/twin/jump', self.on_jump, 1)
            self.get_logger().info(f'AGV-style /cmd_vel rehearsal -> UDP {self.forwarder.address}')

        def on_twist(self, message):
            try:
                self.forwarder.twist(message.linear.x, message.angular.z)
            except (ValueError, OSError) as error:
                self.get_logger().error(f'UDP Twist rejected: {error}')

        def on_jump(self, _message):
            try:
                self.forwarder.jump()
            except (ValueError, OSError) as error:
                self.get_logger().error(f'UDP jump rejected: {error}')

        def destroy_node(self):
            self.forwarder.close()
            return super().destroy_node()

    rclpy.init(args=args)
    node = None
    try:
        node = Bridge()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
