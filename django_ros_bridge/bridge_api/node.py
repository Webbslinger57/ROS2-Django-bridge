import queue
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import importlib
import numpy as np
import base64

"""
This class is a ROS2 node that is used to get a list of all
ROS2 nodes, topics, and services as well as their types.
"""
class ROS2Node(Node):
    # Initialize the ROS2 node.
    def __init__(self):
        super().__init__('django_bridge_node')
        self.incoming_msgs = queue.Queue(maxsize=20)
        self.images = queue.Queue(maxsize=10)
        self.spin_node_thread = threading.Thread(target=self.spin_node)
        self.spin_node_thread.start()
        
    # Spin the ROS2 node.
    def spin_node(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                time.sleep(0.5)
                # Send from outgoing queue...
        except rclpy.executors.ExternalShutdownException:
            pass
        finally:
            self.deinit()
        
    # Deinitialize the ROS2 node.
    def deinit(self):
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        
    # Import a message type from a string
    def get_message_type(self, message_type_str):
        module_name, message_name = message_type_str.replace('/', '.').rsplit('.', 1)
        module = importlib.import_module(module_name)
        return getattr(module, message_name)
        
    # Handles the callback for a subscription
    def sub_callback(self, msg):
        if self.incoming_msgs.full():
            self.incoming_msgs.get()
        self.incoming_msgs.put(msg)

    # Adds a subscription to the node
    def add_subscription(self, topic_endpoint, msg_type):
        type = self.get_message_type(msg_type)
        self.create_subscription(type, topic_endpoint, self.sub_callback, 10)
        
    # Make a service call and return the response
    def call_service(self, service_endpoint, msg_type, msg):
        type = self.get_message_type(msg_type)
        print(type)
       
    # Convert Header to Dictionary
    def header_to_dict(self, header):
        return {
            'stamp': header.stamp.sec + header.stamp.nanosec / 1e9,
            'frame_id': header.frame_id
        }
       
    # Convert an image to a base64 string
    def image_to_base64(self, image: Image):
        # Convert the image data to a numpy array
        image_data = np.array(image, dtype=np.uint8)

        # Convert the numpy array to bytes
        image_bytes = image_data.tobytes()

        # Encode the bytes to a base64 string
        base64_str = base64.b64encode(image_bytes).decode('utf-8')

        return base64_str
       
    # Convert a message to a dictionary
    def msg_to_dict(self, msg):
        msg_dict = {}
        for field_name in msg.get_fields_and_field_types():
            field_value = getattr(msg, field_name)
            if field_name == 'data' and isinstance(msg, Image):
                if self.images.full():
                    self.images.get()
                img = {"frame": str(msg.header.frame_id), "data": self.image_to_base64(field_value)}
                self.images.put(img)
                field_value = "In Image Queue"
            if isinstance(field_value, Header):
                field_value = self.header_to_dict(field_value)
            msg_dict[field_name] = field_value
        return msg_dict
        
    # Get the imcoming messages
    def get_incoming_msgs(self):
        msgs = []
        while not self.incoming_msgs.empty():
            msg = self.incoming_msgs.get()
            msg_dict = self.msg_to_dict(msg)
            msgs.append(msg_dict)
        return msgs