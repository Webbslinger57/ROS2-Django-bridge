from rest_framework.views import APIView
from rest_framework.response import Response
import rclpy
from rclpy.node import Node

"""
This class is a ROS2 node that is used to get a list of all
ROS2 nodes, topics, and services as well as their types.
"""
class ROS2Node(Node):
    def __init__(self):
        super().__init__('django_bridge_node')

"""
This class is a Django APIView that returns a list of 
all ROS2 nodes, topics, and services as well as their types.
"""
class ROS2APIView(APIView):
    def get(self, request, format=None):
        rclpy.init(args=None)
        node = ROS2Node()
        
        # Get a list of all nodes
        nodes = node.get_node_names()
        # Get a list of all topics
        topics = node.get_topic_names_and_types()
        # Get a list of all services
        services = node.get_service_names_and_types()
        
        node.destroy_node()
        rclpy.shutdown()

        return Response({
            'nodes': nodes,
            'topics': topics,
            'services': services,
        })

class ROS2TopicAPIView(APIView):
    def get(self, request, topic_name, format=None):
        rclpy.init(args=None)
        node = ROS2Node(topic_name)

        rclpy.spin_once(node)

        response = {
            'topic': topic_name,
            'data': node.latest_msg,
        }

        node.destroy_node()
        rclpy.shutdown()

        return Response(response)