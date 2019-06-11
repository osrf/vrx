#!/usr/bin/env python
import os
import rospy
import numpy as np
import yaml
from .. import utils


class Sensor_Compliance:
    def __init__(self):
        # open sensor_compliance_visual.sdf and all the boxes defined => boxes
        self.boxes = find_boxes('sensor_compliance_visual.sdf')
        # look at all sensors in sensors directory and get the default params
        self.sensors_dir = rospy.get_param('sensors_dir') + '/'
        self.default_parameters = utils.get_macros(self.sensors_dir)
        self.dir = rospy.get_param('compliance_dir') + '/'
        self.numeric = yaml.load(open(self.dir +
                                      'sensor_compliance_numeric.yaml'))
        return

    def param_compliance(self, sensor_type, params={}):
        # ie: given an instance of sensor_type = 'wamv_camera'
        # with parameters = params, is this camera in compliance
        # check if the sensor is allowed
        assert sensor_type in self.default_parameters,\
                '%s is not defined anywhere under %s' % (sensor_type, self.dir)
        # add the default params to params if not specified
        for i in self.default_parameters[sensor_type]:
            if i not in params:
                params[i] = self.default_parameters[sensor_type][i]
        # right now the ONLY compliance check we have is to make sure that
        # the sensors are in at least one of the boxes
        if 'x' and 'y' and 'z' in params:
            xyz = np.array([float(params['x']),
                            float(params['y']),
                            float(params['z'])])
            for box in self.boxes:
                if box.fit(xyz):
                    return True
            print '\n', sensor_type, params['name'], 'is out of bounds\n'
            return False
        else:
            return True

    def number_compliance(self, sensor_type, n):
        # ie: are n wamv_cameras allowed?
        if n > self.numeric[sensor_type]:
            print '\n', 'maximum of', self.numeric[sensor_type], sensor_type, \
                'allowed\n'
            return False
        return True


class Thruster_Compliance:
    def __init__(self):
        return

    def param_compliance(self, thruster_type, params={}):
        # ie: given an instance of thruster_type = 'wamv_camera'
        # with parameters = params, is this camera in compliance
        return True

    def number_compliance(self, thruster_type, n):
        # ie: are n wamv_cameras allowed?
        return True


class Box:
    def __init__(self, pose, size):
        self.pose = np.array([float(j) for j in [i for i in pose.split(' ')
                              if i != '']])
        self.size = np.array([float(j) for j in [i for i in size.split(' ')
                              if i != '']])
        return

    def fit(self, pose):
        pose = pose-self.pose[:3]
        for idx, i in enumerate(pose):
            if abs(i) > self.size[idx]/2:
                return False
        return True


def find_boxes(sdf):
    addrs = rospy.get_param('compliance_dir') + '/' + sdf
    sdf = open(addrs, 'r')
    sdf = sdf.read()
    boxes = []
    while sdf.find('<visual') != -1:
        start = sdf.find('<visual')
        sdf = sdf[start+7:]

        start = sdf.find('<pose')
        sdf = sdf[start+5:]

        start = sdf.find('>')
        sdf = sdf[start+1:]
        end = sdf.find('<')
        pose = sdf[:end]
        sdf = sdf[end:]

        start = sdf.find('<geometry')
        sdf = sdf[start:]

        start = sdf.find('<box>')
        sdf = sdf[start:]

        start = sdf.find('<size>')
        end = sdf.find('</')
        size = sdf[start+6:end]
        boxes.append(Box(pose, size))
    return boxes
