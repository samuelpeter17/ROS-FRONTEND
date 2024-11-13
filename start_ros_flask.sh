# start_ros_flask.sh
#!/bin/bash

# Exit on any error
set -e

# Source the ROS setup
source /opt/ros/melodic/setup.bash
source /root/catkin_ws/devel/setup.bash

# Create .ros directory for gazebo
mkdir -p ~/.ros

# Start the virtual framebuffer for X11
Xvfb :99 -screen 0 1280x720x24 -ac +extension GLX +render -noreset &
sleep 2

# Start x11vnc with debugging and better error handling
x11vnc -display :99 -forever -shared -nopw -xkb -noxrecord -noxfixes -noxdamage -rfbport 5900 &
sleep 2

# Start roscore
roscore &
sleep 5

# Set Gazebo model path
export GAZEBO_MODEL_PATH=/opt/ros/melodic/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH

# Launch Gazebo without GUI
roslaunch turtlebot3_gazebo turtlebot3_world.launch gui:=false &
sleep 10

# Start the Flask application
cd /root/catkin_ws
exec python3 app.py