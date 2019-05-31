#!/usr/bin/env python
class Compliance:
    def __init__(self):
        return
    def param_compliance(self, sensor_type, params={}):
        # an instance of 'wamv_camera' with params, is this camera in compliance
        return True


    def sensor_number_compliance(self, sensor_type, n):
        # ie: are n wamv_cameras allowed?
        return True
