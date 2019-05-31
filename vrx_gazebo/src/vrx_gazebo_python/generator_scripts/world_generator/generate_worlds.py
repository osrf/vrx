#!/usr/bin/env python
import yaml
import rospy
import os
from collections import OrderedDict


from .. utils import macro_block_gen

def main():

    s = open(rospy.get_param('requested'))
    master = yaml.load(s)
    
    a = linear_combinations(master)
    print len(a)
    
    '''
    a = world_gen(coordinates={'environment':0}, master=master)
    macro_block_gen(target = rospy.get_param('world_xacro_target') + 'world1.world.xacro',
                    available = rospy.get_param('available'),
                    requested_macros = a,
                    boiler_plate_top = '<?xml version="1.0" ?>\n<sdf version="1.6" xmlns:xacro="http://ros.org/wiki/xacro">\n<world name="robotx_example_course">\n<xacro:include filename="$(find vrx_gazebo)/worlds/sandisland.xacro" />\n',
                    boiler_plate_bot = '</world>\n</sdf>')
    os.system('rosrun xacro xacro --inorder -o' +
              rospy.get_param('world_target') + 'world1.world ' +
              rospy.get_param('world_xacro_target')+ 'world1.world.xacro')
    '''



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

def linear_combinations(master = {}):
    combinations = []
    axies = OrderedDict()
    start = {}
    for axis, value in master.iteritems():
        start[axis] = 0
        axies[axis] = value['steps']-1

    iterate(axies_max = axies, coordinates = combinations, current_coordinate = start)
    return combinations


def iterate(axies_max={}, coordinates=[], current_coordinate = {}):
    coordinates.append(current_coordinate.copy())
    if axies_max == current_coordinate:
        return
    else:
        for axis in current_coordinate:
            if current_coordinate[axis] < axies_max[axis]:
                current_coordinate[axis] += 1
                break
            else:
                current_coordinate[axis] = 0
        iterate(axies_max=axies_max, coordinates = coordinates, current_coordinate = current_coordinate)





