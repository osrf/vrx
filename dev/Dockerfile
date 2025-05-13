# Ubuntu 24.04 
FROM osrf/ros:jazzy-desktop-full

# Setup timezone
ENV TZ=Etc/UTC
RUN echo $TZ > /etc/timezone && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    apt update && \
    apt install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Set up repo to install Gazebo
RUN curl -s https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

  # Install some 'standard' ROS packages and utilities.
RUN . /opt/ros/jazzy/setup.sh \
  && apt update \
  && apt install -y --no-install-recommends \
     gz-harmonic \
     python3-colcon-common-extensions \
     python3-vcstool \
     ros-${ROS_DISTRO}-ackermann-msgs \
     ros-${ROS_DISTRO}-actuator-msgs \
     ros-${ROS_DISTRO}-ament-cmake-pycodestyle \
     ros-${ROS_DISTRO}-image-transport \
     ros-${ROS_DISTRO}-image-transport-plugins \
     ros-${ROS_DISTRO}-joy-teleop \
     ros-${ROS_DISTRO}-joy-linux \
     ros-${ROS_DISTRO}-mavros-msgs \
     ros-${ROS_DISTRO}-navigation2 \
     ros-${ROS_DISTRO}-nav2-bringup \
     ros-${ROS_DISTRO}-nav2-minimal-tb3-sim \
     ros-${ROS_DISTRO}-nav2-minimal-tb4-description\
     ros-${ROS_DISTRO}-nav2-minimal-tb4-sim \
     ros-${ROS_DISTRO}-radar-msgs \
     ros-${ROS_DISTRO}-ros-gz-sim\
     ros-${ROS_DISTRO}-vision-msgs \
     ros-${ROS_DISTRO}-xacro \
  && rm -rf /var/lib/apt/lists/* \
  && apt clean -qq

# Tools necessary and useful during development
RUN apt update && \
 DEBIAN_FRONTEND=noninteractive \
 apt install --no-install-recommends -y \
        atop \
        expect \
        gdb \
        iputils-ping \
        gnutls-bin \
        libbluetooth-dev \
        libcwiid-dev \
        net-tools \
        python3-dbg \
        python3-pip \
        python3-venv \
        vim \
        wget \
        xvfb \
&& rm -rf /var/lib/apt/lists/* \
&& apt clean -qq
