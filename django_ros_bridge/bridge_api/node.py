import queue
import threading
import rclpy
from rclpy.node import Node
import importlib

"""
This class is a ROS2 node that is used to get a list of all
ROS2 nodes, topics, and services as well as their types.
"""
class ROS2Node(Node):
    # Initialize the ROS2 node.
    def __init__(self):
        super().__init__('django_bridge_node')
        self.incoming_msgs = queue.Queue(maxsize=100)
        self.spin_node_thread = threading.Thread(target=self.spin_node)
        self.spin_node_thread.start()
        
    # Spin the ROS2 node.
    def spin_node(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                # Process messages...
        except rclpy.executors.ExternalShutdownException:
            pass
        finally:
            self.deinit()
        
    # Deinitialize the ROS2 node.
    def deinit(self):
        self.node.destroy_node()
        rclpy.shutdown()
        
    # Import a message type from a string
    def get_message_type(self, message_type_str):
        module_name, message_name = message_type_str.replace('/', '.').rsplit('.', 1)
        module = importlib.import_module(module_name)
        return getattr(module, message_name)
        
    # Handles the callback for a subscription
    def sub_callback(self, msg):
        print(msg)
        if self.incoming_msgs.full():
            self.incoming_msgs.get()
        self.incoming_msgs.put(msg)

    # Adds a subscription to the node
    def add_subscription(self, topic_endpoint, msg_type):
        type = self.get_message_type(msg_type)
        self.create_subscription(type, topic_endpoint, self.sub_callback, 10)
        print(self.subscriptions)
        
    # Make a service call and return the response
    def call_service(self, service_endpoint, msg_type, msg):
        type = self.get_message_type(msg_type)
        print(type)
        
    # Get the imcoming messages
    def get_incoming_msgs(self):
        msgs = []
        while not self.incoming_msgs.empty():
            msg = self.incoming_msgs.get()
            msgs.append(msg.data) # Change This To Work With All Message Types
        return msgs