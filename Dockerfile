# Base image - Ubuntu 20.04 with ROS Noetic
FROM osrf/ros:noetic-desktop-full-focal

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Install basic system utilities and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    lsb-release \
    python3-pip \
    python3-dev \
    python3-setuptools \
    libusb-1.0-0-dev \
    udev \
    wget \
    curl \
    vim \
    nano \
    gdb \
    iputils-ping \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Install ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-effort-controllers \
    ros-${ROS_DISTRO}-position-controllers \
    ros-${ROS_DISTRO}-joy \
    && rm -rf /var/lib/apt/lists/*

# Install Dynamixel SDK
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-dynamixel-workbench \
    ros-${ROS_DISTRO}-dynamixel-workbench-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Install computer vision dependencies
RUN apt-get update && apt-get install -y \
    libopencv-dev \
    python3-opencv \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

# Install RealSense SDK and ROS packages
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
    add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo focal main" -u && \
    apt-get update && apt-get install -y \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --upgrade pip && \
    pip3 install \
    numpy \
    scipy \
    scikit-learn \
    matplotlib \
    pandas \
    pillow \
    torch \
    torchvision \
    tensorflow-cpu \
    imutils \
    mediapipe \
    pyrealsense2

# Create workspace structure
RUN mkdir -p /workspace/src

# Set up environment variables
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/devel/setup.bash 2>/dev/null || true" >> ~/.bashrc

# Set working directory
WORKDIR /workspace

# Set up udev rules for RealSense and Dynamixel
RUN mkdir -p /etc/udev/rules.d
COPY setup/99-realsense-libusb.rules /etc/udev/rules.d/ 
COPY setup/99-dynamixel.rules /etc/udev/rules.d/

# Expose ROS Master port
EXPOSE 11311

# Copy entry point script
COPY docker-entrypoint.sh /entrypoint.sh
# Set entry point
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]