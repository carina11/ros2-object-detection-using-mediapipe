FROM ros:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    nano \
    python3-pip \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rviz2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROBOT_WS=/robot_ws
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Fix Rviz black screen issue ROS-2372 https://github.com/ros2/rviz/issues/948
RUN apt update && \
    apt install -y software-properties-common && \
    add-apt-repository ppa:kisak/kisak-mesa && \
    apt install -y mesa-utils && \
    apt upgrade -y &&\
    apt clean && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir mediapipe tensorflow opencv-python scikit-learn pandas seaborn

# install OpenCV dependecies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ffmpeg libsm6 libxext6 ros-${ROS_DISTRO}-cv-bridge && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./robot_ws/src $ROBOT_WS/src/

# To avoid permission issue
ARG UNAME=user
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o $UNAME
RUN useradd -m -u $UID -g $GID -o -s /bin/bash $UNAME
RUN usermod -aG sudo $UNAME
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER $UNAME
RUN sudo chown -R $UNAME $ROBOT_WS

RUN cd $ROBOT_WS && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    sudo apt-get update && \
    rosdep update && rosdep install --from-path . --ignore-src -r -y && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/* && \
    colcon build --symlink-install

RUN echo '. /opt/ros/$ROS_DISTRO/setup.sh' >> ~/.bashrc && \
    echo '. $ROBOT_WS/install/setup.bash' >> ~/.bashrc

ENV ENTRYPOINT_PATH=./docker/ros_entrypoint.sh
COPY $ENTRYPOINT_PATH /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
