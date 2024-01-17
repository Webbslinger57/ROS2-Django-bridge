echo "Launchinc django_ros_bridge..."
echo "Sourcing ROS2..."
source /opt/ros/humble/setup.bash # Source ROS2
echo "Sourcing ros2 workspace..."
echo "Loading..."
source $HOME/ros2_ws/install/setup.bash # Source your ros2 workspace

# Ignore this part
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
echo "|| | | | |    |  |   / _ \   |   \|  || |      | |  | | ||"
echo "|| | | | | _  |  |  / /_\ \  |    \  || |   __ | |  | | ||"
echo "|| | |_| || |_|  | /  ___  \ |  |\   || |__|_ || |__| | ||"
echo "|| |____/ |______|/__/   \__\|__| \__| \_____/  \____/  ||"
echo " \======================================================/ "
echo "Starting django_ros_bridge..."
echo "Loading..."
SECONDS=0
while [ $SECONDS -lt 2 ]; do
    for i in / - \\ \|; do
        echo -ne "\r$i"
        sleep 0.1
    done
done
echo ""
# Until here..."It's for fun" - Nacho

# Remove '0.0.0.0:8000' if you want to run it on localhost
python3 django_ros_bridge/manage.py runserver 0.0.0.0:8000 # Run django server
