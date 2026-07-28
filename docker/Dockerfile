# VRX 4 development image
# ROS 2 Lyrical Luth (Ubuntu 26.04 "Resolute") + Gazebo Jetty.
#
# Unlike the legacy image, this does NOT fake the gz vendor packages or build
# ros_gz from source. VRX 4's gz_waves packages `find_package(gz_sim_vendor)`,
# so we install the REAL ros-<distro>-gz-*-vendor packages (they carry the CMake
# shims and pull in Gazebo Jetty). This image only provides the environment;
# build the VRX workspace at runtime in the mounted ~/vrx_ws.
FROM ros:lyrical-ros-base AS vrx-base

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# ---- Locale + base tooling ------------------------------------------------
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg lsb-release locales tzdata \
 && ln -fs /usr/share/zoneinfo/$TZ /etc/localtime \
 && echo $TZ > /etc/timezone \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ---- OSRF Gazebo apt repo -------------------------------------------------
# Provides Gazebo Jetty and the libs the gz_*_vendor packages depend on.
RUN curl -s https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

# ---- Gazebo Jetty + ROS gz vendor shims + ros_gz --------------------------
# The gz-*-vendor packages provide the CMake shims that VRX 4's gz_waves
# packages find_package(). The quoted glob is passed literally to apt, which
# expands it to every ros-<distro>-gz-<x>-vendor package (msgs, transport,
# fuel-tools, sim, rendering, ...). Gazebo itself arrives via these vendor
# packages + ros-gz; Resolute (26.04) has no standalone `gz-jetty` metapackage
# in the OSRF repo, and installing one alongside the vendors would be redundant.
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-gz \
      "ros-${ROS_DISTRO}-gz-*-vendor" \
      ros-${ROS_DISTRO}-sdformat-vendor \
 && rm -rf /var/lib/apt/lists/*

# ---- Standard ROS packages + build tooling VRX uses -----------------------
# Source of truth for VRX's runtime dependencies is the package.xml files +
# rosdep; this hand-maintained list only PRIMES the image layer for cache reuse
# (the source tree isn't in the image, so `rosdep install --from-paths src` runs
# later against the mounted workspace and is what actually resolves deps). Keep
# this roughly in sync with the rosdep keys, or regenerate it from package.xml.
# NOTE: nav2 (navigation2, nav2-bringup, nav2-minimal-*) is intentionally
# omitted — it is often not yet released early in a new distro's life. Add
# ros-${ROS_DISTRO}-navigation2 etc. once available on Lyrical.
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ackermann-msgs \
      ros-${ROS_DISTRO}-actuator-msgs \
      ros-${ROS_DISTRO}-ament-cmake-pycodestyle \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-image-transport-plugins \
      ros-${ROS_DISTRO}-joy-teleop \
      ros-${ROS_DISTRO}-joy-linux \
      ros-${ROS_DISTRO}-mavros-msgs \
      ros-${ROS_DISTRO}-radar-msgs \
      ros-${ROS_DISTRO}-vision-msgs \
      ros-${ROS_DISTRO}-xacro \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
 && rm -rf /var/lib/apt/lists/*

# ---- Dev tools + NVIDIA GL (rocker --nvidia) ------------------------------
# 64-bit GL/X11 runtime libs only; VRX (Gazebo Jetty) is 64-bit, so no i386.
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      atop expect gdb gnutls-bin iputils-ping libbluetooth-dev libcwiid-dev \
      net-tools python3-dbg python3-pip python3-venv vim wget xvfb \
      libxau6 libxdmcp6 libxcb1 libxext6 libx11-6 libglvnd0 \
      libgl1 libglx0 libegl1 libgles2 \
 && rm -rf /var/lib/apt/lists/*

# ---- Source ROS for interactive + non-interactive shells ------------------
# BASH_ENV below makes every non-interactive `bash -c` source the ROS env too,
# so `run_compose.bash <cmd>` and `docker exec ... bash -c` get ros2 on PATH.
# Caveat: this sources setup.bash even under scripts that run `set -u`, which
# can trip on unbound ROS vars — guard such a script with a localized `set +u`
# around its ROS calls if needed.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" > /etc/profile.d/ros.sh \
 && chmod +x /etc/profile.d/ros.sh \
 && echo "source /etc/profile.d/ros.sh" >> /etc/bash.bashrc
ENV BASH_ENV=/etc/profile.d/ros.sh

WORKDIR /ws
CMD ["/bin/bash"]
