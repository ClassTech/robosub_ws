FROM ros:jazzy-ros-base

# Install system and ROS2 dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-flask \
    python3-numpy \
    python3-opencv \
    python3-pygame \
    ros-jazzy-cv-bridge \
    ros-jazzy-launch-ros \
    # SDL2 libs needed by pygame
    libsdl2-dev \
    libsdl2-image-dev \
    libsdl2-ttf-dev \
    libgl1-mesa-dri \
    libglib2.0-0 \
    # Virtual display + VNC
    xvfb \
    x11vnc \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /robosub_ws

COPY src/ src/
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --packages-select robosub

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 5900 8080

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "robosub", "robosub.launch.py"]
