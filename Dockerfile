FROM osrf/ros:melodic-desktop-full

# Install necessary dependencies and ROS packages
RUN apt-get -y update && apt-get install -y \
    curl \
    iputils-ping \
    net-tools \
    wget \
    screen \
    git \
    nano \
    vim \
    htop \
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
    python3-pip \
    python3-dev \
    ros-melodic-turtlebot3-gazebo \
    ros-melodic-rosbridge-server \
    ros-melodic-web-video-server \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install \
    flask \
    flask-restful \
    pyyaml \
    rospkg \
    pillow \
    numpy

# Create the ROS workspace
RUN mkdir -p /root/catkin_ws/src

# Set the working directory to catkin_ws
WORKDIR /root/catkin_ws

# Set up the ROS environment
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash"

# Copy the source code into the workspace (if applicable)
#COPY src/ /root/catkin_ws/src/

# Copy the app.py file from the project root into the container's /root/catkin_ws
#COPY app.py /root/catkin_ws/app.py

# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"

# Set up ROS environment so it's sourced on every new shell session
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Copy the start_ros_flask.sh script and make it executable
COPY start_ros_flask.sh /root/start_ros_flask.sh
RUN chmod +x /root/start_ros_flask.sh

# Set the default command to execute the start script
CMD ["/root/start_ros_flask.sh"]
