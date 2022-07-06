#!/usr/bin/env python3

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

import argparse
import csv
import math
import os
import pathlib
import subprocess
import sys

import numpy as np


def parse_args():
    parser = argparse.ArgumentParser('Set wave and wind parameters.')
    parser.add_argument('--sea-state', dest='sea_state', type=str,
                        help='Set sea state [0-2]')
    parser.add_argument('--date', dest='hist_date', type=str,
                        help='Set sea state using historic weather')
    parser.add_argument('--wind-dir', dest='wind_dir', nargs='+',
                        help='Set wind direction vector [x y z]')
    args = parser.parse_args()
    return args


def update_params(wave_gain, wave_period, wind_speed_v):
    wind_speed_v_str = ' '.join(str(w) for w in wind_speed_v)

    process = subprocess.Popen(
        ['sh', 'update_sea_state_params.sh', str(wave_gain), str(wave_period),
            str(wind_speed_v_str)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()
    if stderr is not None and len(stderr) != 0:
        print(stdout, stderr)
        sys.exit(1)


def update_params_by_date(hist_date, wind_dir):
    wave_height = None
    wave_period = None
    wind_speed_kmh = None

    csv_file_name = 'weather_data_hourly.csv'
    csv_file_path = os.path.join(pathlib.Path(__file__).parent.resolve(), 'weather')
    csv_file_path = os.path.join(csv_file_path, csv_file_name)

    # read from csv spreadsheet file
    with open(csv_file_path, 'rt') as csv_file:
        reader = csv.DictReader(filter(lambda r: r[0] != '#', csv_file),
                                delimiter=',')
        for row in reader:
            if hist_date in row['date']:
                wave_height = float(row['wave'])
                wave_period = float(row['period'])
                wind_speed_kmh = float(row['wind'])
                print(f'Found weather data for date:{hist_date}')
                print(f'  wave height (m):{wave_height}')
                print(f'  wave period (s):{wave_period}')
                print(f'  wind speed (km/h):{wind_speed_kmh}')
                break

    if wave_height is None or wave_period is None or wind_speed_kmh is None:
        print(f'No weather data found for date:{hist_date}.')
        print(f'Make sure the specified date exists in {csv_file_path}',
              'and it is in the format: [yyyymmdd]T[hhmm], e.g. 20201225T1200')
        sys.exit(1)

    # convert wind speed from km/h to m/s
    wind_speed = wind_speed_kmh / 3.6

    # hardcode these wavefield variables
    # todo: grab from world / model file?
    number = 3
    scale = 1.1
    # amplitude is half of total wave height
    amplitude = wave_height * 0.5

    # compute wave gain using PMS model
    gain = gain_from_pms(amplitude, wave_period, scale, number)

    wind_speed_v = np.array(wind_dir) * wind_speed

    print(f'Setting sea state by date:{hist_date}')
    print(f'    wave gain (avg):{gain}')
    print(f'    wave period:{wave_period}')
    print(f'    wind linear velocity:{wind_speed_v}')

    update_params(gain, wave_period, wind_speed_v)


def update_params_by_sea_state(sea_state, wind_dir):
    try:
        sea_state_code = int(sea_state)
    except ValueError:
        print(f"Error converting sea state code '{sea_state}' to integer")
        sys.exit(1)

    gain = None
    wind_speed = None
    wave_period = 5.0
    if sea_state_code == 0:
        gain = 0.0
        wind_speed = 0.0
    elif sea_state_code == 1:
        gain = 0.3
        wind_speed = 0.5
    elif sea_state_code == 2:
        gain = 0.5
        wind_speed = 2
    else:
        print(f'Sea state: {sea_state} not supported. Value must be [0-2].')

    wind_speed_v = wind_dir * wind_speed
    print(f'Setting sea sate by code:{sea_state}')
    print(f'    wave gain:{gain}')
    print(f'    wave period:{wave_period}')
    print(f'    wind linear velocity:{wind_speed_v}')

    update_params(gain, wave_period, wind_speed_v)


def run_main():
    args = parse_args()

    # Get wind direction and normalize vector
    wind_dir = None
    if not args.wind_dir:
        wind_dir = [1, 0, 0]
    else:
        try:
            wind_dir = np.array(list(map(float, args.wind_dir)))
        except ValueError:
            print(f'Error converting wind direction {args.wind_dir} to floating point array')
            sys.exit(1)
    if len(wind_dir) != 3:
        print('Wind direction vector must be in the form of [x y z]')
        sys.exit(1)

    wind_dir = wind_dir / np.linalg.norm(wind_dir)

    # update params
    if args.hist_date is not None and len(args.hist_date) != 0:
        update_params_by_date(args.hist_date, wind_dir)
    elif args.sea_state is not None and len(args.sea_state) != 0:
        update_params_by_sea_state(args.sea_state, wind_dir)

    print('Done setting sea state params')


def pm(omega, omegaP):
    alpha = 0.0081
    g = 9.81
    pm = alpha * pow(g, 2.0) / pow(omega, 5.0) * math.exp(-(5.0 / 4.0) * pow(omegaP / omega, 4.0))
    return pm


def gain_from_pms(amplitude, period, scale, number):
    angular_frequency = 2.0 * math.pi / period
    os0 = angular_frequency * (1.0 - 1.0 / scale)
    os1 = angular_frequency * (scale - 1.0 / scale) / 2.0
    os2 = angular_frequency * (scale - 1.0)
    omega_spacing = [os0, os1, os2]

    avg_gain = 0.0
    for i in range(number):
        n = i - 1
        scale_factor = pow(scale, n)
        omega = angular_frequency * scale_factor
        pms = pm(omega, angular_frequency)

        # solve for gain
        # amplitude = gain * sqrt(2.0 * pms * omegaSpacing[i])
        gain = amplitude / math.sqrt(2.0 * pms * omega_spacing[i])
        avg_gain += gain

    avg_gain = avg_gain / number
    return avg_gain


if __name__ == '__main__':
    run_main()
