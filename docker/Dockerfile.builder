FROM vrx-base AS vrx-builder

COPY . /ws/src/

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt update \
    && rosdep install -r \
      --from-paths /ws/src/ \
      --ignore-src \
      --rosdistro ${ROS_DISTRO} -y \
      --skip-keys="gz_math_vendor gz_msgs_vendor gz_sim_vendor gz_transport_vendor gz_sim_vendor gz_msgs_vendor gz_transport_vendor gz_msgs_vendor gz_transport_vendor"


# Build the project
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd /ws \
    && colcon build --symlink-install --merge-install
