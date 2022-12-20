#!/bin/sh

# Copyright 2022 Open Source Robotics Foundation, Inc.
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

if [ "$#" -eq  "0" ]
then
  echo "Usage: ./update_sea_state_params.sh [wave_gain] [wave_period] [wind_speed]"
  exit 1
fi

wave_gain=$1
wave_period=$2
wind_speed=$3

DIR="$( cd "$( dirname "$0" )" && pwd )"

# update wave gain param
find $DIR/../models -type f -exec sed -i ':a;N;$!ba;s/\(<wavefield>.*\)<gain>.*<\/gain>\(.*<\/wavefield\)/\1<gain>'"$wave_gain"'<\/gain>\2/g' {} \;
find $DIR/../worlds -type f -exec sed -i ':a;N;$!ba;s/\(<wavefield>.*\)<gain>.*<\/gain>\(.*<\/wavefield\)/\1<gain>'"$wave_gain"'<\/gain>\2/g' {} \;


# update wave period param
find $DIR/../models -type f -exec sed -i ':a;N;$!ba;s/\(<wavefield>.*\)<period>.*<\/period>\(.*<\/wavefield\)/\1<period>'"$wave_period"'<\/period>\2/g' {} \;
find $DIR/../worlds -type f -exec sed -i ':a;N;$!ba;s/\(<wavefield>.*\)<period>.*<\/period>\(.*<\/wavefield\)/\1<period>'"$wave_period"'<\/period>\2/g' {} \;

# update wind linear vel param
find $DIR/../worlds -type f -exec sed -i ':a;N;$!ba;s/\(<wind>.*\)<linear_velocity>.*<\/linear_velocity>\(.*<\/wind\)/\1<linear_velocity>'"$wind_speed"'<\/linear_velocity>\2/g' {} \;
