from rest_framework.views import APIView
from rest_framework.response import Response

import rclpy
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
This view gets information about a specific ROS2 topic.
"""
class ROS2TopicAPIView(APIView):    
    def get(self, request, format=None):
        topic_endpoint = request.GET.get('topic_endpoint', None)
        
        # Get a list of all topics
        topics = node.get_topic_names_and_types()
        
        # Create a dictionary where the keys are the topic names and the values are the types
        topic_dict = {topic[0]: topic[1] for topic in topics}
        
        if topic_endpoint not in topic_dict:
            return Response({
                'status': '400',
                'error': 'Topic not found.',
            })
            
        type = topic_dict[topic_endpoint]
        
        return Response({
            'status': '200',
            'topic': topic_endpoint,
            'message_type': type,
        })
        
        # Add Subscription
        # node.add_subscription(topic_endpoint, 'std_msgs/msg/String')
