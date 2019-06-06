#!/usr/bin/env python
class Sensor_Compliance:
    def __init__(self):
        return

    def param_compliance(self, sensor_type, params={}):
        # ie: given an instance of sensor_type = 'wamv_camera'
        # with parameters = params, is this camera in compliance
        return True

    def number_compliance(self, sensor_type, n):
        # ie: are n wamv_cameras allowed?
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
