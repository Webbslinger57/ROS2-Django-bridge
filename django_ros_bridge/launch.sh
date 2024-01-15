echo "Launchinc django_ros_bridge..."
echo "Sourcing ROS2..."
source /opt/ros/humble/setup.bash
echo "Sourcing ros2 workspace..."
source $HOME/ros2_ws/install/setup.bash
echo "===================================================="
echo " ____      ____     _      __   __   _____    ____  "
echo "|  _ \     |  |    / \    |  \ |  | /  ___|  / __ \ "
echo "| | | |    |  |   / - \   |   \|  ||  /     | |  | |"
echo "| | | | _  |  |  /  _  \  |    \  || |   __ | |  | |"
echo "| |_| || |_|  | /  / \  \ |  |\   ||  \_|_ || |__| |"
echo "|____/ |______|/__/   \__\|__| \__| \_____/  \____/ "
echo "===================================================="
echo "Starting django_ros_bridge..."
python3 manage.py runserver 0.0.0.0:8000
