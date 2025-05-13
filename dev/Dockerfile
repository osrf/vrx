# Ubuntu 24.04 
FROM osrf/ros:jazzy-desktop-full

ARG ROSDIST=jazzy

# Setup timezone
ENV TZ=Etc/UTC
RUN echo $TZ > /etc/timezone && \
  ln -fs /usr/share/zoneinfo/$TZ /etc/localtime
  
# Tools necessary and useful during development
RUN apt update && \
 DEBIAN_FRONTEND=noninteractive \
 apt install --no-install-recommends -y \
        build-essential \
        atop \
	ca-certificates \
        cmake \
        cppcheck \
	curl \
        expect \
        gdb \
        git \
	gnupg2 \
        gnutls-bin \
	iputils-ping \
        libbluetooth-dev \
        libccd-dev \
        libcwiid-dev \
	libeigen3-dev \
        libfcl-dev \
        libgflags-dev \
        libgles2-mesa-dev \
        libgoogle-glog-dev \
        libspnav-dev \
        libusb-dev \
	lsb-release \
	net-tools \
	pkg-config \
        protobuf-compiler \
        python3-dbg \
        python3-empy \
        python3-numpy \
        python3-setuptools \
        python3-pip \
        python3-venv \
	ruby \
        software-properties-common \
	sudo \
        vim \
	wget \
	xvfb \
 && apt clean -qq

# Setup locale
RUN sudo apt update && sudo apt install locales \
  && sudo locale-gen en_US en_US.UTF-8 \
  && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Set up repo to install Gazebo
RUN /bin/sh -c 'wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null'

  # Install some 'standard' ROS packages and utilities.
RUN apt update \
  && apt install -y --no-install-recommends \
     gz-harmonic \
     python3-colcon-common-extensions \
     python3-vcstool \
     ros-${ROSDIST}-ackermann-msgs \
     ros-${ROSDIST}-actuator-msgs \
     ros-${ROSDIST}-ament-cmake-pycodestyle \
     ros-${ROSDIST}-image-transport \
     ros-${ROSDIST}-image-transport-plugins \
     ros-${ROSDIST}-joy-teleop \
     ros-${ROSDIST}-joy-linux \
     ros-${ROSDIST}-mavros-msgs \
     ros-${ROSDIST}-navigation2 \
     ros-${ROSDIST}-nav2-bringup \
     ros-${ROSDIST}-nav2-minimal-tb3-sim \
     ros-${ROSDIST}-nav2-minimal-tb4-description\
     ros-${ROSDIST}-nav2-minimal-tb4-sim \
     ros-${ROSDIST}-radar-msgs \
     ros-${ROSDIST}-ros-gz-sim\
     ros-${ROSDIST}-vision-msgs \
     ros-${ROSDIST}-xacro \    
  && sudo rm -rf /var/lib/apt/lists/* \
  && sudo apt clean -qq

