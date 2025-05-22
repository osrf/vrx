# VRX Docker support

For running VRX using the docker support please refer to the instructions
in https://github.com/osrf/vrx/wiki/docker_install_tutorial.

## Dockerfiles in this directory

The directory contains a series of Dockerfiles designed to serve different setups
for VRX. The usage is mainly controlled by `docker-compose` although docker files
can be used independently if desired:

 * `Dockerfile.base`: built on top of osrf desktop-full. Install
   gz-harmonic from packages.osrfoundation.org repository and
   the VRX dependencies using rosdep. Generates the `vrx-base` image.

   * `Dockerfile.builder`: built on top of `vrx-base`. Compiles the vrx packages
     using colcon. Generates the `vrx-builder` image.

   * `Dockerfile.devel`: built on top of `vrx-base`. development image with
     extra ROS 2 development packages and other dev utilities. It configures
     the GPU support and a development user. Runs a shell into the container
     with local sources mapped in /ws. Generates the `vrx-devel` image.
