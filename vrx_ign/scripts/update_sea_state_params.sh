#!/bin/sh

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

# # update wind linear vel param
find $DIR/../worlds -type f -exec sed -i ':a;N;$!ba;s/\(<wind>.*\)<linear_velocity>.*<\/linear_velocity>\(.*<\/wind\)/\1<linear_velocity>'"$wind_speed"'<\/linear_velocity>\2/g' {} \;
