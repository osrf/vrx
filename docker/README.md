# VRX Docker support

# Overview

The directory contains a series of Dockerfiles designed to serve different setups
for VRX. The usage is mainly controlled by `docker-compose` although docker files
can be used independently if desired:

 * `Dockerfile.base`: built on top of osrf desktop-full. Install
   gz-harmonic from packages.osrfoundation.org repostiory and
   the VRX dependencies using rosdep. Generates the `vrx-base` image.

   * `Dockerfile.builder`: built on top of `vrx-base`. Compiles the vrx packages
     using colcon. Generates the `vrx-builder` image.

   * `Dockerfile.devel`: built on top of `vrx-base`. development image with
     extra ROS 2 development packages and other dev utilities. It configures
     the GPU support and a development user. Runs a shell into the container
     with local sources mapped in /ws. Generates the `vrx-devel` image.

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
# check that no more dependencies needs to be installed and compile vrx code
$ ubuntu@localhost:/ws$ sudo apt update \
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
