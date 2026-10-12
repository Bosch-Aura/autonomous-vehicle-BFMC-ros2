import pytest
from bfmc_example.example_node import ExampleNode


def test_node_name():
    """El nodo se crea con el nombre correcto."""
    import rclpy
    rclpy.init()
    node = ExampleNode()
    assert node.get_name() == 'example_node'
    node.destroy_node()
    rclpy.shutdown()
