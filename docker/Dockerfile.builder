FROM vrx-base AS vrx-builder

COPY . /ws/src/

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt update \
    && rosdep install -r \
      --from-paths /ws/src/ \
      --ignore-src \
      --rosdistro ${ROS_DISTRO} -y

# Build the project
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd /ws \
    && colcon build --symlink-install --merge-install
