# Django-ROS2-Bridge

This package acts as a web interface for a ROS2 system. It will contain both REST API Endpoints as well as regular webpages.

## Django-Overview

- The django portion of this package will contain two apps:
    1. bridge_api
        - This app will provide various REST API endpoints for interacting with the ROS2 system.
    2. bridge_interface
        - This app will provide similar functionality, but in the form of rendered web pages that can be used in a mobile app or viewed from a web browser on the same network.

## ROS2-Overview

- This package is built using ROS2 Humble and python.

## Getting Started

- Ensure you have django, django-rest, and ros2-humble dependencies installed. 
- Navigate to the django_ros_bridge directory

```bash
cd django_ros_bridge/
```

- source the setup script

```bash
source setup.sh
```

- run the server 

```bash
# localhost
python3 manage.py runserver
# Expose to network
python3 manage.py runserver 0.0.0.0
```