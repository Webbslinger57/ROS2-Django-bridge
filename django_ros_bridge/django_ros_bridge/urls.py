from django.contrib import admin
from django.urls import path
from bridge_api.views import ROS2APIView #, ROS2TopicAPIView, ROS2ServiceAPIView

urlpatterns = [
    path('admin/', admin.site.urls),
    path('ros2/', ROS2APIView.as_view(), name='ros2'),
    # path('ros2/topic/<str:topic_name>/', ROS2TopicAPIView.as_view(), name='ros2_topic'),
    # path('ros2/service/<str:service_name>/', ROS2ServiceAPIView.as_view(), name='ros2_service'),
]