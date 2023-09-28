FROM ros:humble

ENV ROS_DISTRO humble

# install ros package.
RUN apt-get update && apt-get install -y \
    python3-pip \
    curl \      
    && rm -rf /var/lib/apt/lists/*

RUN sudo apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rosbag2-storage-mcap \
    ros-$ROS_DISTRO-rosbag2 \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-ros2bag \
    ros-$ROS_DISTRO-rosbag2-transport \
    && rm -rf /var/lib/apt/lists/*

# sourcing ROS
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source install/local_setup.bash" >> /home/$USERNAME/.bashrc


WORKDIR /workspaces/soft_landing

COPY src src
COPY ros2_entrypoint.sh ros2_entrypoint.sh

RUN colcon build

# download rosbridge for Foxglove monitoring
RUN pip install pyvectorguidance
RUN apt update && apt-get install -y ros-humble-rosbridge-suite 


RUN pip install --no-cache-dir  citros

RUN chmod +x ros2_entrypoint.sh
ENTRYPOINT ["/workspaces/soft_landing/ros2_entrypoint.sh"]

CMD ["bash"]