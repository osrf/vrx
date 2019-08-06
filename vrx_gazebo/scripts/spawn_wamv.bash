#!/bin/bash

# spawn_wamv.bash: A bash script to spawn a wamv model using gz.
#
# E.g.: ./spawn_wamv.bash /home/<username>/my_urdf.urdf

set -e

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Define usage function.
usage()
{
  echo "Usage: $0 <urdf_abs_path>"
  exit 1
}

# Call usage() function if arguments not supplied.
echo "*******************************#$##"
echo "Has $# parameters"
# [[ $# -ne 1 ]] && usage

sleep 20s
echo "DONE SLEEP"
gz model --model-name=wamv --spawn-file=$HOME/my_urdf.urdf
