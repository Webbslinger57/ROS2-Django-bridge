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
                'error': 'Topic not found.',
            }, status=400)
            
        type = topic_dict[topic_endpoint]
        
        return Response({
            'topic': topic_endpoint,
            'message_type': type,
        }, status=200)
        
    def post(self, request, format=None):
        topic_endpoint = request.GET.get('topic_endpoint', None)
        
        # Get a list of all topics
        topics = node.get_topic_names_and_types()
        
        # Create a dictionary where the keys are the topic names and the values are the types
        topic_dict = {topic[0]: topic[1] for topic in topics}
        
        if topic_endpoint not in topic_dict:
            return Response({
                'error': 'Topic not found.',
            }, status=400)
            
        type = topic_dict[topic_endpoint]
        
        # Add Subscription
        node.add_subscription(topic_endpoint, type)
        
        return Response({
            'topic': topic_endpoint,
            'message_type': type,
        }, status=200)

class ROS2ServiceAPIView(APIView):
    def get(self, request, format=None):
        service_endpoint = request.GET.get('service_endpoint', None)
        
        # Get a list of all services
        services = node.get_service_names_and_types()
        
        # Create a dictionary where the keys are the service names and the values are the types
        service_dict = {service[0]: service[1] for service in services}
        
        if service_endpoint not in service_dict:
            return Response({
                'error': 'Service not found.',
            }, status=400)
            
        type = service_dict[service_endpoint]
        
        return Response({
            'service': service_endpoint,
            'message_type': type,
        }, status=200)
        
    def post(self, request, format=None):
        service_endpoint = request.GET.get('service_endpoint', None)
        
        # Get a list of all services
        services = node.get_service_names_and_types()
        
        # Create a dictionary where the keys are the service names and the values are the types
        service_dict = {service[0]: service[1] for service in services}
        
        if service_endpoint not in service_dict:
            return Response({
                'error': 'Service not found.',
            }, status=400)
            
        type = service_dict[service_endpoint]

        # Make service call and return response
        node.call_service(service_endpoint, type, request.data)
        return Response({
            'service': service_endpoint,
            'message_type': type,
        }, status=200)
        