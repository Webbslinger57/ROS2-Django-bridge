echo "Launchinc django_ros_bridge..."
echo "Sourcing ROS2..."
source /opt/ros/humble/setup.bash
echo "Sourcing ros2 workspace..."
echo "Loading..."
source $HOME/ros2_ws/install/setup.bash
SECONDS=0
while [ $SECONDS -lt 2 ]; do
    for i in / - \\ \|; do
        echo -ne "\r$i"
        sleep 0.1
    done
done
echo ""
echo " /======================================================\ "
echo "||           ____     ____     _____   ______           || "
echo "||          |  _ \   / __ \   /   __| |___   \          || "
echo "||          | |_| | | |  | |  |  |        |  |          || "
echo "||          |    /  | |  | |   \  \      /  /           || "
echo "||          | |\ \  | |__| |  __|  |    /  /__          || "
echo "||          |_| \_\  \____/  |_____/   |______|         || "
echo " \====================             =====================/ "
echo "      ~          ~~  \\\            \\\  ~     ~~~         "
echo "            ~~~~      \\\            \\\      ~~           "
echo "     ~~                ||            ||          ~~      "
echo "         ~~~           ||            ||   ~~~~           "
echo "               ~~      ||            ||       ~~         "
echo "                   ~~ //            //   ~~       ~      "
echo "      ~          ~~~~//            // ~~       ~~~~      "
echo " /====================             =====================\ "
echo "||  ____      ____     _      __   __   _____    ____   ||"
echo "|| |  _ \     |  |    / \    |  \ |  | / ____|  / __ \  ||"
echo "|| | | | |    |  |   / - \   |   \|  || |      | |  | | ||"
echo "|| | | | | _  |  |  /  _  \  |    \  || |   __ | |  | | ||"
echo "|| | |_| || |_|  | /  / \  \ |  |\   || |__|_ || |__| | ||"
echo "|| |____/ |______|/__/   \__\|__| \__| \_____/  \____/  ||"
echo " \======================================================/ "
echo "Starting django_ros_bridge..."
echo "Loading..."
source $HOME/ros2_ws/install/setup.bash
SECONDS=0
while [ $SECONDS -lt 2 ]; do
    for i in / - \\ \|; do
        echo -ne "\r$i"
        sleep 0.1
    done
done
echo ""
python3 django_ros_bridge/manage.py runserver 0.0.0.0:8000
