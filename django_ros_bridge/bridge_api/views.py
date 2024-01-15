from rest_framework.views import APIView
from rest_framework.response import Response
import rclpy
from rclpy.node import Node

class ROS2Node(Node):
    def __init__(self):
        super().__init__('ros2_node')

class ROS2APIView(APIView):
    def get(self, request, format=None):
        rclpy.init(args=None)
        node = ROS2Node()

        # Get a list of all topics
        topics = node.get_topic_names_and_types()
        # Get a list of all services
        services = node.get_service_names_and_types()

        node.destroy_node()
        rclpy.shutdown()

        return Response({
            'topics': topics,
            'services': services,
        })