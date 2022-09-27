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

import codecs
import subprocess
import time

def monitor_sim():
    # wait a few secs before starting to pgrep for process
    time.sleep(10)
    quit = False
    # monitor gazebo process until it exits
    while not quit:
        time.sleep(1)
        process = subprocess.Popen(['pgrep', '-f', 'gz sim -v 4'],
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)
        stdout = process.communicate()[0]
        str_output = codecs.getdecoder('unicode_escape')(stdout)[0]
        if len(str_output) == 0:
            quit = True


if __name__ == '__main__':
    monitor_sim()
