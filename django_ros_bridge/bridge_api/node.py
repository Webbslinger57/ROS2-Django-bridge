import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32

"""
This class is a ROS2 node that is used to get a list of all
ROS2 nodes, topics, and services as well as their types.
"""
class ROS2Node(Node):
    # Initialize the ROS2 node.
    def __init__(self):
        self.msg_types = { # Dictionary of message types and their callbacks
            'std_msgs/msg/String': [String, self.str_callback],
            'std_msgs/msg/Int32': [Int32, self.int_callback],
        }
        self._subscriptions = {}
        self._publishers = {}
        super().__init__('django_bridge_node')
        
    def deinit(self):
        self.node.destroy_node()
        rclpy.shutdown()
        
    # Handles string subscriptions
    def str_callback(self, msg):
        print(msg.data)
        return msg.data

    # Handles int subscriptions
    def int_callback(self, msg):
        print(msg.data)
        return msg.data

    # Adds a subscription to the node
    def add_subscription(self, topic_endpoint, msg_type):
        type = self.msg_types[msg_type][0]
        callback = self.msg_types[msg_type][1]
        self.create_subscription(type, topic_endpoint, callback, 10)
        print(self.subscriptions)