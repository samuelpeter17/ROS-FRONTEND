#!/bin/bash

# Ensure the script uses LF line endings, even on Windows-based systems (this is handled by .gitattributes)

# Source the ROS setup
source /opt/ros/melodic/setup.bash

# Start roscore in the background
roscore &

# Wait for roscore to initialize (adjust sleep time if needed)
sleep 5  

# Run the Flask app
python3 /root/catkin_ws/app.py
