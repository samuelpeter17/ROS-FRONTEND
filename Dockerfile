# Dockerfile
FROM osrf/ros:melodic-desktop-full

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    curl \
    iputils-ping \
    net-tools \
    wget \
    screen \
    git \
    nano \
    vim \
    htop \
    python3-pip \
    python3-dev \
    x11vnc \
    xvfb \
    websockify \
    novnc \
    ros-melodic-joy \
    ros-melodic-teleop-twist-joy \
    ros-melodic-teleop-twist-keyboard \
    ros-melodic-laser-proc \
    ros-melodic-rgbd-launch \
    ros-melodic-depthimage-to-laserscan \
    ros-melodic-rosserial-arduino \
    ros-melodic-rosserial-python \
    ros-melodic-rosserial-server \
    ros-melodic-rosserial-client \
    ros-melodic-rosserial-msgs \
    ros-melodic-amcl \
    ros-melodic-map-server \
    ros-melodic-move-base \
    ros-melodic-urdf \
    ros-melodic-xacro \
    ros-melodic-compressed-image-transport \
    ros-melodic-rqt-image-view \
    ros-melodic-gmapping \
    ros-melodic-navigation \
    ros-melodic-interactive-markers \
    ros-melodic-turtlebot3-gazebo \
    ros-melodic-rosbridge-server \
    ros-melodic-web-video-server \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    x11vnc \
    xvfb \
    mesa-utils \
    xorg \
    openbox \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*  
      
# Install Python packages using pip
RUN pip3 install \
    flask \
    flask-restful \
    pyyaml \
    rospkg

# Create catkin workspace
WORKDIR /root/catkin_ws

# Copy your ROS package source code
COPY src/your_ros_package /root/catkin_ws/src

# Build the workspace
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"

# Copy the Flask app and start script
COPY app.py /root/catkin_ws/app.py
COPY start_ros_flask.sh /root/start_ros_flask.sh

# Make the start script executable
RUN chmod +x /root/start_ros_flask.sh

# Command to run the start script
CMD ["/root/start_ros_flask.sh"]