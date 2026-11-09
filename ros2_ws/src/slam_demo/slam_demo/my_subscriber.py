import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class PositionSubscriber(Node):
    def __init__(self):
        super().__init__('position_subscriber')

        # create the subscriber: message type, topic name, function to call, queue size
        self.subscription = self.create_subscription(
            Float64, 'robot_x', self.on_position, 10)

    def on_position(self, message):
        self.get_logger().info(f'Heard x = {message.data}')


def main():
    rclpy.init()
    node = PositionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()