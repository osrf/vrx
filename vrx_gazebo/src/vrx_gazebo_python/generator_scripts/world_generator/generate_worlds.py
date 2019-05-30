#!/usr/bin/env python
import yaml
import rospy

from .. utils import macro_block_gen

s = open(rospy.get_param('requested'))
master = yaml.load(s)



def world_gen(coordinates = {}, master = {}):
    world = {}
    for axis_name, axis in master.iteritems():
        for macro_name, macros in axis['macros'].iteritems():
            world[macro_name] = []
            for params in macros:
                if params != None:
                    for param, value in params.iteritems():
                        params[param] = (lambda n: eval(value))(coordinates[axis_name])
                    world[macro_name].append(params)
                else:
                    world[macro_name].append({})
    return world

a = world_gen(coordinates={'environment':0}, master=master)
macro_block_gen(target = rospy.get_param('target')+'world1.world.xacro',
                available = rospy.get_param('available'),
                requested_macros = a,
                boiler_plate_top = '<?xml version="1.0" ?>\n<sdf version="1.6" xmlns:xacro="http://ros.org/wiki/xacro">\n<world name="robotx_example_course">\n<xacro:include filename="$(find vrx_gazebo)/worlds/sandisland.xacro" />\n',
                boiler_plate_bot = '  </wolrd>\n</sdf>')



