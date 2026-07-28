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
#

# Builds a Docker image.
# Usage: build.bash <dir containing Dockerfile> [tag]
#   tag defaults to the resolved directory name, so passing '.' works too.
#
# For the VRX dev container, build it as vrx_dev:lyrical so it matches the
# image referenced in compose.yaml:
#   ./build.bash <dir containing Dockerfile> lyrical
image_name=vrx_dev

if [ $# -lt 1 ]
then
    echo "Usage: $0 <path to directory containing Dockerfile> [tag]"
    exit 1
fi

context="$1"
if [ ! -f "${context}/Dockerfile" ]
then
    echo "Err: ${context} does not contain a Dockerfile to build."
    exit 1
fi

# Tag: explicit 2nd arg, else the resolved directory basename (handles '.').
distro="${2:-$(basename "$(cd "${context}" && pwd)")}"

image_plus_tag=$image_name:$(export LC_ALL=C; date +%Y_%m_%d_%H%M)
docker build --rm -t "$image_plus_tag" -f "${context}/Dockerfile" "${context}" && \
docker tag "$image_plus_tag" "$image_name:$distro" && \
echo "Built $image_plus_tag and tagged as $image_name:$distro" && \
echo "To run:  ./docker/run_compose.bash"
