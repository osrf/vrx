# Ubuntu 20.04 with nvidia-docker2 beta opengl support.
ARG BASEIMG=ros:noetic-ros-base
FROM $BASEIMG

# Set Ubuntu release
ARG RELEASE=focal

# Set ROS distribution
ARG DIST=noetic

# Set Gazebo verison
ARG GAZ=gazebo11

# Tools useful during development.
RUN apt update \
 && apt install -y --no-install-recommends\
        build-essential \
        cppcheck \
        curl \
        cmake \
        lsb-release \
        gdb \
        git \
        libeigen3-dev \
        pkg-config \
        protobuf-compiler \
        python3-dbg \
        python3-pip \
        python3-venv \
        ruby \
        software-properties-common \
        sudo \
        vim \
        wget \
 && apt clean

# Fixing TimeZone problem reported in VRX 2019
RUN export DEBIAN_FRONTEND=noninteractive \
 && apt update \
 && apt install -y \
    tzdata \
 && ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && apt clean

# Get ROS melodic and Gazebo 11.
#COPY docker/keys/gazebo.key /tmp/gazebo.key
#COPY docker/keys/ros.key /tmp/ros.key
RUN /bin/sh -c 'echo "deb http://packages.ros.org/ros/ubuntu ${RELEASE} main" > /etc/apt/sources.list.d/ros1-latest.list' \
 && /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable ${RELEASE} main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
 && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743 \
 && apt update \
 && apt install -y --no-install-recommends \
    ${GAZ} \
    lib${GAZ}-dev \
    libgles2-mesa-dev \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    ros-${DIST}-gazebo-ros \
    ros-${DIST}-gazebo-ros-control \
    ros-${DIST}-gazebo-ros-pkgs \
    ros-${DIST}-gazebo-plugins \
    ros-${DIST}-hector-gazebo-plugins \
    ros-${DIST}-joint-state-controller \
    ros-${DIST}-joint-state-publisher \
    ros-${DIST}-joy \
    ros-${DIST}-joy-teleop \
    ros-${DIST}-key-teleop \
    ros-${DIST}-robot-localization \
    ros-${DIST}-robot-state-publisher \
    ros-${DIST}-ros-base \
    ros-${DIST}-rqt \
    ros-${DIST}-rqt-common-plugins \
    ros-${DIST}-rviz \
    ros-${DIST}-teleop-tools \
    ros-${DIST}-teleop-twist-keyboard \
    ros-${DIST}-tf2-geometry-msgs \
    ros-${DIST}-tf2-tools \
    ros-${DIST}-velodyne-simulator \
    ros-${DIST}-xacro \
    qtbase5-dev \
 && apt clean

# RUN rosdep update

# Set USER and GROUP
#ARG USER=developer
#ARG GROUP=developer
#
## Add a user with the same user_id as the user outside the container
## Requires a docker build argument `user_id`.
#
#RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
#    chown root:root /usr/local/bin/fixuid && \
#    chmod 4755 /usr/local/bin/fixuid && \
#    mkdir -p /etc/fixuid && \
#    printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml
#
#RUN addgroup --gid 1000 $USER && \
#    adduser --uid 1000 --ingroup $USER --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER
#
#RUN adduser $USER sudo \
# && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER
#
## Commands below run as the developer user.
#USER $USER:$GROUP
#
## When running a container start in the developer's home folder.
#WORKDIR /home/$USER
#
## Create workspace
#RUN mkdir -p vrx_ws/src/vrx
#
## Copy the VRX repository from the local file system
## We can't use the USER:GROUP variables until Docker adds support to --chown
#COPY --chown=developer:developer . vrx_ws/src/vrx/
#
## Compile the VRX project.
#RUN /bin/bash -c ". /opt/ros/${DIST}/setup.bash && cd vrx_ws && catkin_make"
#
## Source all the needed environment files.
#RUN /bin/sh -c 'echo ". /opt/ros/${DIST}/setup.bash" >> ~/.bashrc' \
# && /bin/sh -c 'echo ". /usr/share/gazebo/setup.sh" >> ~/.bashrc' \
# && /bin/sh -c 'echo ". ~/vrx_ws/devel/setup.sh" >> ~/.bashrc'
#
#ENTRYPOINT ["fixuid"]
#
#CMD ["/bin/bash"]
## Customize your image here.
#
#
## ...
