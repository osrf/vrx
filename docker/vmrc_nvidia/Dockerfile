# Ubuntu 16.04 with nvidia-docker2 beta opengl support.
FROM nvidia/opengl:1.0-glvnd-devel-ubuntu16.04

# Tools useful during development.
RUN apt-get update \
 && apt-get install -y \
        build-essential \
        curl \
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

# Get ROS kinetic and Gazebo 7.
RUN /bin/sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
 && /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
 && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -' \
 && apt-get update \
 && apt-get install -y \
    ros-kinetic-desktop-full \
    ros-kinetic-hector-gazebo-plugins \
    ros-kinetic-joy \
    ros-kinetic-joy-teleop \
    ros-kinetic-robot-localization \
    ros-kinetic-velodyne-simulator \
    gazebo7 \
    libgazebo7-dev \
 && rosdep init \
 && apt-get clean

RUN rosdep update

# Set USER and GROUP 
ARG USER=developer
ARG GROUP=developer

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`.

RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml

RUN addgroup --gid 1000 $USER && \
    adduser --uid 1000 --ingroup $USER --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER

RUN adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

# Commands below run as the developer user.
USER $USER:$GROUP

# When running a container start in the developer's home folder.
WORKDIR /home/$USER

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

ENTRYPOINT ["fixuid"]

CMD ["/bin/bash"]
# Customize your image here.
# ...
