import rclpy
from rclpy.node import Node


class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        self.get_logger().info('example_node iniciado')


def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
