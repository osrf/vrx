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

is_gzserver_running()
{
  if pgrep gzserver >/dev/null; then
    true
  else
    false
  fi
}

wait_until_gzserver_is_up()
{
  until is_gzserver_running
  do
    sleep 2s
  done

  while [[ "$(gz topic -l | wc -l)" -le 2 ]];do
    sleep 2s
  done

  sleep 2s
}

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

wait_until_gzserver_is_up
echo "gzserver is up"
gz model --model-name=wamv --spawn-file=$HOME/my_urdf.urdf
