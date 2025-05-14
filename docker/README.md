# VRX Docker support

# Overview

The directory contains a series of Dockerfiles designed to serve different setups
for VRX. The usage is mainly controlled by `docker-compose` although docker files
can be used independently if desired:

 * `Dockerfile.base`: built on top of osrf desktop-full. Install
   gz-harmonic from packages.osrfoundation.org repostiory and
   the VRX dependencies using rosdep.

   * `Dockerfile.builder`: built on top of `vrx-base`. Compiles the vrx packages
     using colcon.

     * `Dockerfile.release`: built on top of `vrx-builder`. Install the NVIDIA
     setup to run GPU and an entrypoint to source ROS setup.bash and workspace
     setup.bash.

   * `Dockerfile.devel`: built on top of `vrx-base`. development image with extra ROS 2 development packages
     and other dev utilities. It configures the GPU support and a development
     user. Runs a shell into the container with local sources mapped in /ws.


# Development container

The system is prepared to leave a running container with the
VRX setup ready and the local clone of this repository mapped
into the container in /ws (using docker volumes).

The docker compose will perform:

 * Build the necessary docker images
 * Prepare the NVIDIA setup to run the container with GPU support
 * Use `ubuntu` user for development

A typical working run can consist on:

```bash
$ cd vrx/docker
$ docker-compose run --rm devel
...
... 
# docker building and return a shell inside the container
$ ubuntu@locashot:/ws$ export ROS_DISTRO=jazzy
# compile vrx code
$ ubuntu@localhost:/ws$ . /opt/ros/${ROS_DISTRO}/setup.sh \
  && apt update \
  && rosdep install -r --from-paths src/ --ignore-src --rosdistro ${ROS_DISTRO} -y 
$ ubuntu@locashot:/ws$ colcon build --merge-install
... # colcon compilation happen
$ ubuntu@locashot:/ws$ . install/setup.bash
$ ubuntu@locashot:/ws$ ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

# TODO

 * Use the existing images infrastructure for releasing
 * Use the existing images infrastructure for CI
 * Use no root to build the source in vrx-builder

## Development container

 * Replace the ubuntu user by UID/GUID of existing system user
 * Support for Intel GPU systems (enable /dev/dri in compose)
 * Provide an entrypoint that runs rosdep install
