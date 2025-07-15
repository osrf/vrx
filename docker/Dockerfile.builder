FROM vrx-base AS vrx-builder

COPY . /ws/src/

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt update \
    && rosdep install -r \
      --from-paths /ws/src/ \
      --ignore-src \
      --rosdistro ${ROS_DISTRO} -y \
      --skip-keys="gz_sim_vendor gz_physics_vendor gz_launch_vendor gz_sensors_vendor sdformat_vendor ros_gz_sim_demos ros_gz_bridge ros_gz_interfaces ros_gz_sim" \
    && rm -rf /var/lib/apt/lists/* \
    && apt clean -qq

# Build the project
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && . ~/ros_gz_ws/install/setup.sh \
    && cd /ws \
    && colcon build --symlink-install --merge-install