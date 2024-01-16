from django.contrib import admin
from django.urls import path
from bridge_api.views import ROS2APIView, ROS2TopicAPIView, ROS2ServiceAPIView, ROS2IncomingMsgAPIView

urlpatterns = [
    path('admin/', admin.site.urls),
    path('ros2/', ROS2APIView.as_view(), name='ros2'),
    path('ros2/topic/', ROS2TopicAPIView.as_view(), name='ros2_topic'),
    path('ros2/service/', ROS2ServiceAPIView.as_view(), name='ros2_service'),
    path('ros2/incoming_msg/', ROS2IncomingMsgAPIView.as_view(), name='ros2_incoming_msg'),
]