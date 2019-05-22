#!/usr/bin/env python

import os
import yaml

from utils import get_macros, Macro
from compliance import param_compliance
xacro_file=open('../wamv_description/urdf/sensors.xacro', 'wb')
xacro_file.write('<?xml version="1.0"?>\n')
xacro_file.write('<robot xmlns:xacro="http://ros.org/wiki/xacro" name="wam-v-sensors">\n')
xacro_file.write('<xacro:macro name="sensors">\n')

avalible_macros = get_macros('../wamv_gazebo/urdf/sensors/')

s = open('sensor_config.yaml','r')
requested_macros = yaml.load(s)

for key, devices in requested_macros.items():
    #device must be avalible
    assert key in avalible_macros.keys(),"%s is not defined in %s"%(key, directory)
    assert sensor_number_compliance(key, len(devices)),"%d %s's are not allowed"%(len(devices),key)
    xacro_file.write('  <!-- === %s === -->\n'% (key))
    for i in devices:
        macro_call='  <xacro:%s '%key
        full_params = avalible_macros[key].params.copy()
        for j in i:
            #all params in all devices must be correct
            assert j in avalible_macros[key].params.keys(),"%s is not a parameter in %s"%(j,key)
            macro_call+='%s="%s" '%(j, str(i[j]))
            full_params[j] = i[j]
        assert param_compliance(key, full_params),"%s %s failed compliance test"%(key, i['name'])
        macro_call+='/>\n'
        xacro_file.write(macro_call)

xacro_file.write('</xacro:macro>\n')
xacro_file.write('</robot>\n')
xacro_file.close()


cmd = "roslaunch vrx_gazebo sandisland.launch"
os.system(cmd)
