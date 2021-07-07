#!/usr/bin/env bash

#
# Copyright (C) 2018 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# Runs a docker container with the image created by build.bash
# Requires:
#   docker
#   nvidia-docker
#   an X server
#   rocker
# Recommended:
#   A joystick mounted to /dev/input/js0 or /dev/input/js1

IMG_NAME=$1
JOY=/dev/input/js0
# Split off ':latest' from IMG_NAME
IFS=':' read -ra NAMES <<< "$IMG_NAME"
CONTAINER_NAME="${NAMES[0]}_runtime"
echo "Using image <$IMG_NAME> to start container <$CONTAINER_NAME>"

rocker --devices $JOY --dev-helpers --nvidia --x11 --user --home --name $CONTAINER_NAME --git $IMG_NAME
