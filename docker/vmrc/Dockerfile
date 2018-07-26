# Ubuntu 16.04 with nvidia-docker2 beta opengl support.
FROM nvidia/opengl:1.0-glvnd-devel-ubuntu16.04

# Tools useful during development.
RUN apt-get update \
 && apt-get install -y \
        build-essential \
        cmake \
        lsb-release \
        gdb \
        git \
        mercurial \
        python3-dbg \
        python3-pip \
        python3-venv \
        ruby \
        software-properties-common \
        sudo \
        vim \
        wget \
 && apt-get clean

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`.
ARG user_id
ENV USERNAME developer
RUN useradd -U --uid ${user_id} -ms /bin/bash $USERNAME \
 && echo "$USERNAME:$USERNAME" | chpasswd \
 && adduser $USERNAME sudo \
 && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

# Commands below run as the developer user.
USER $USERNAME

# When running a container start in the developer's home folder.
WORKDIR /home/$USERNAME

# Get ROS kinetic and Gazebo 7.
RUN sudo /bin/sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
 && sudo /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
 && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -' \
 && sudo apt-get update \
 && sudo apt-get install -y \
    ros-kinetic-desktop-full \
    ros-kinetic-joy-teleop \
    ros-kinetic-robot-localization \
    gazebo7 \
    libgazebo7-dev \
 && sudo rosdep init \
 && sudo apt-get clean

RUN rosdep update

# Clone the VMRC repository.
RUN mkdir -p vmrc_ws/src \
 && cd vmrc_ws \
 && hg clone https://bitbucket.org/osrf/vmrc src/vmrc

# Compile the VMRC project.
RUN /bin/bash -c ". /opt/ros/kinetic/setup.bash && cd vmrc_ws && catkin_make"

# Source all the needed environment files.
RUN /bin/sh -c 'echo ". /opt/ros/kinetic/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". /usr/share/gazebo/setup.sh" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". ~/vmrc_ws/devel/setup.sh" >> ~/.bashrc'

# Customize your image here.
# ...
