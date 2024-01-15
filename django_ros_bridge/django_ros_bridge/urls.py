from django.contrib import admin
from django.urls import path
from bridge_api.views import ROS2APIView

urlpatterns = [
    path('admin/', admin.site.urls),
    path('ros2/', ROS2APIView.as_view(), name='ros2'),
]