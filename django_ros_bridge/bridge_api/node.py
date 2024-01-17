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
This class is the ROS2 node that acts as a proxy between 
the Django server and the ROS2 network. It is responsible
for subscribing to topics, calling services, and sending
messages to the Django server.
"""
class ROS2Node(Node):
    def __init__(self):
        """Initialize the ROS2 node."""
        super().__init__('django_bridge_node')
        self.incoming_msgs = queue.Queue(maxsize=20)
        self.images = queue.Queue(maxsize=10)
        self.spin_node_thread = threading.Thread(target=self.spin_node)
        self.spin_node_thread.start()

    def spin_node(self):
        """Spin the ROS2 node."""
        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                time.sleep(0.5)
                # Send from outgoing queue...
        except rclpy.executors.ExternalShutdownException:
            pass
        finally:
            self.deinit()

    def deinit(self):
        """Deinitialize the ROS2 node."""
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def get_message_type(self, message_type_str):
        """Get a ROS message type from a string."""
        module_name, message_name = message_type_str.replace('/', '.').rsplit('.', 1)
        module = importlib.import_module(module_name)
        return getattr(module, message_name)

    def sub_callback(self, msg):
        """Callback for a subscription."""
        if self.incoming_msgs.full():
            self.incoming_msgs.get()
        self.incoming_msgs.put(self.msg_to_dict(msg))

    def add_subscription(self, topic_endpoint, msg_type):
        """Add a subscription to the node."""
        type = self.get_message_type(msg_type)
        self.create_subscription(type, topic_endpoint, self.sub_callback, 10)
        
    # TODO: Not implemented yet
    def call_service(self, service_endpoint, msg_type, msg):
        """Call a ROS service."""
        type = self.get_message_type(msg_type)
        print(type)
       
    def header_to_dict(self, header):
        """Convert a ROS Header to a dictionary."""
        return {
            'stamp': header.stamp.sec + header.stamp.nanosec / 1e9,
            'frame_id': header.frame_id
        }
       
    def image_to_base64(self, image: Image):
        """Convert an image to a base64 string."""
        image_data = np.array(image, dtype=np.uint8)
        image_bytes = image_data.tobytes()
        base64_str = base64.b64encode(image_bytes).decode('utf-8')

        return base64_str
       
    def msg_to_dict(self, msg):
        """Convert ROS message to a dictionary."""
        msg_dict = {}
        for field_name in msg.get_fields_and_field_types():
            field_value = getattr(msg, field_name)
            
            # Convert image and store in image queue
            if field_name == 'data' and isinstance(msg, Image):
                if self.images.full():
                    self.images.get()
                img = {"frame": str(msg.header.frame_id), "data": self.image_to_base64(field_value)}
                self.images.put(img)
                field_value = "Image In Queue"
            if isinstance(field_value, Header):
                field_value = self.header_to_dict(field_value)
            msg_dict[field_name] = field_value
        return msg_dict
        
    def get_incoming_msgs(self):
        """Get incoming messages."""
        msgs = []
        while not self.incoming_msgs.empty():
            msg_dict = self.incoming_msgs.get()
            # msg_dict = self.msg_to_dict(msg)
            msgs.append(msg_dict)
        return msgs