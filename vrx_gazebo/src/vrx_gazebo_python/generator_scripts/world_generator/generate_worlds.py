#!/usr/bin/env python
import yaml
import rospy
import os
from collections import OrderedDict


from .. utils import macro_block_gen

def main():

    s = open(rospy.get_param('requested'))
    master = yaml.load(s)
    
    coordinates = linear_combinations(master)

    for num,i in enumerate(coordinates):
        macro_block_gen(target = rospy.get_param('world_xacro_target') + 'world' + str(num) + '.world.xacro',
                        available = rospy.get_param('available'),
                        requested_macros = world_gen(coordinate=i, master=master),
                        boiler_plate_top = '<?xml version="1.0" ?>\n<sdf version="1.6" xmlns:xacro="http://ros.org/wiki/xacro">\n<world name="robotx_example_course">\n<xacro:include filename="$(find vrx_gazebo)/worlds/xacros/include_all_xacros.xacro" />\n',
                        boiler_plate_bot = '</world>\n</sdf>')
        os.system('rosrun xacro xacro --inorder -o' +
                  rospy.get_param('world_target') + 'world' + str(num) + '.world ' +
                  rospy.get_param('world_xacro_target')+ 'world' + str(num) + '.world.xacro')
    



def world_gen(coordinate = {}, master = {}):
    world = {}
    for axis_name, axis in master.iteritems():
        if coordinate[axis_name] in axis['sequence']:
            world.update(axis['sequence'][coordinate[axis_name]])
            break
        for macro_name, macro_calls in axis['macros'].iteritems():
            world[macro_name] = []
            for params in macro_calls:
                if params != None:
                    for param, value in params.iteritems():
                        params[param] = (lambda n: eval(value))(coordinate[axis_name])
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





