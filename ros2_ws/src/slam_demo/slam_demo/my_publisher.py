import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')

        # Step 1: create the publisher: message type, topic name, queue size
        self.publisher = self.create_publisher(Float64, 'robot_x', 10)

        # Step 2: call self.publish_position once every 1.0 seconds
        self.timer = self.create_timer(1.0, self.publish_position)

        self.robot_x = 0.0
        self.declare_parameter('step_size', 1.0)

    def publish_position(self):
        message = Float64()
        message.data = self.robot_x
        self.publisher.publish(message)
        self.get_logger().info(f'Published x = {self.robot_x}')
        
        step_size = self.get_parameter('step_size').get_parameter_value().double_value
        self.robot_x += step_size


def main():
    rclpy.init()
    node = PositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()