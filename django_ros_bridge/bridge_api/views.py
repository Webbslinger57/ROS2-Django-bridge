from rest_framework.views import APIView
from rest_framework.response import Response
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from bridge_api.node import ROS2Node

"""
Initialize the ROS2 node.
"""
if not rclpy.ok():
    rclpy.init(args=None)
node = ROS2Node()

"""
This class is a Django APIView that returns a list of 
all ROS2 nodes, topics, and services as well as their types.
"""
class ROS2APIView(APIView):
    def get(self, request, format=None):
        
        # Get a list of all nodes
        nodes = node.get_node_names()
        # Get a list of all topics
        topics = node.get_topic_names_and_types()
        # Get a list of all services
        services = node.get_service_names_and_types()

        return Response({
            'nodes': nodes,
            'topics': topics,
            'services': services,
        })

"""
This view is used to get the latest message from a
given ROS2 topic.
"""
class ROS2TopicAPIView(APIView):    
    def get(self, request, format=None):
        topic_endpoint = request.GET.get('topic_endpoint', None)
        
        # Add Subscription
        # node.add_subscription(topic_endpoint, 'std_msgs/msg/String')

        return Response({
            'topic': topic_endpoint,
            'message_type': 'std_msgs/msg/String',
        })