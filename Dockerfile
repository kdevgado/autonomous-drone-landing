#Use Ubuntu 22.04 as the base image
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV VENV_PATH=/opt/venv

# Add ROS 2 key and source before installing ROS packages
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Update and install required dependencies
RUN DEBIAN_FRONTEND='noninteractive' apt-get update && apt-get install -y \
    git \
    wget \
    cmake \
    tmux \
    default-jre \
    build-essential \
    python3-colcon-common-extensions \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    at \
    ros-humble-desktop \
    ros-humble-librealsense2 \
    ros-humble-rclpy \
    ros-humble-sensor-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-foxglove-bridge \
    ros-humble-rplidar-ros \
    libopencv-dev \
    python3-opencv \
    python3 \
    python3-pip \
    python3-dev \
    python3-rosdep \
    python3-setuptools \
    python3-venv \
#---------------------------new for mapping
    ros-humble-ament-index-python \
    ros-humble-cartographer-ros \
    ros-humble-launch \
    ros-humble-launch-ros \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-robot-state-publisher \
    ros-humble-topic-tools \
    ros-humble-twist-stamper \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-ament-cmake \
    python3-pynput \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN geographiclib-get-geoids egm96-5

# Create a virtual environment
RUN python3 -m venv $VENV_PATH
 
# Activate virtualen

ENV PATH="$VENV_PATH/bin:$PATH"

RUN $VENV_PATH/bin/pip install --upgrade pip && \
    $VENV_PATH/bin/pip install \
    opencv-python-headless \
    MAVProxy \
    "numpy<2" \
    pyrealsense2 \
    "ultralytics[dev]" \
    empy==3.3.4 \
    catkin_pkg \
    lark-parser \
    #torch==2.1.0+cpu \
    torch==2.1.0 \
    #torchvision==0.16.0+cpu \
    torchvision==0.16.0 \
    -f https://download.pytorch.org/whl/torch_stable.html \
    typing_extensions \
    pyparsing 

# Setting up ros2-humble-desktop
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc && \
     echo "source $VENV_PATH/bin/activate" >> /root/.bashrc

# To get /mavros/rangefinder/rangefinder to work
RUN cat <<EOF > /opt/ros/humble/share/mavros/launch/px4_pluginlists.yaml
/**:
  ros__parameters:
    plugin_denylist:
      # common

      # extras
      - image_pub
      - vibration
      - wheel_odometry

    plugin_allowlist:
    #   - 'sys_*'
EOF

# Setting up librealsense2
RUN cd /opt && \
    git clone https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && \
    mkdir build && \
    cd build && \
    cmake .. -DBUILD_EXAMPLES=false -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && \
    make -j$(nproc) && \
    make install && \
    cp /opt/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/

# Copy source files to /root/ros2_ws
#COPY ./src /root/ros2_ws/src

WORKDIR /ros2_ws

COPY launch_drone.sh /launch_drone.sh
RUN chmod +x /launch_drone.sh
ENTRYPOINT ["/launch_drone.sh"]
#COPY entrypoint.sh /entrypoint.sh
#RUN chmod +x /entrypoint.sh
#ENTRYPOINT ["/entrypoint.sh"]
#CMD ["bash"]

# Set up ROS 2 workspace

#RUN apt-get update
# Initialize and update rosdep
#RUN rosdep init && rosdep update && rosdep install --from-paths src --ignore-src -r -y

#RUN colcon build && source install/setup.bash

