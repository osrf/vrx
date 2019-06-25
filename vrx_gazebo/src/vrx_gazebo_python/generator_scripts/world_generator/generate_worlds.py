#!/usr/bin/env python
import yaml
import rospy
import os
from collections import OrderedDict

from .. utils import create_xacro_file


def main():
    s = open(rospy.get_param('requested'))
    # get the yaml as a dict
    master = yaml.load(s)
    # get lsit of all coordinates that the master dict maps out
    coordinates = linear_combinations(master)
    # create a world xacro and subsiquent world file for each coordinate
    for num, i in enumerate(coordinates):
        create_xacro_file(xacro_target=rospy.get_param('world_xacro_target') +
                          'world' + str(num) + '.world.xacro',

                          requested_macros=world_gen(coordinate=i,
                                                     master=master),
                          boiler_plate_top='<?xml version="1.0" ?>\n' +
                          '<sdf version="1.6" ' +
                          'xmlns:xacro="http://ros.org/wiki/xacro">\n' +
                          '<world name="robotx_example_course">\n' +
                          '  <xacro:include filename="$(find vrx_gazebo)' +
                          '/worlds/xacros/include_all_xacros.xacro" />\n' +
                          '  <xacro:include_all_xacros />\n',
                          boiler_plate_bot='</world>\n</sdf>')
        os.system('rosrun xacro xacro --inorder -o' +
                  rospy.get_param('world_target') + 'world' + str(num) +
                  '.world ' +
                  rospy.get_param('world_xacro_target') + 'world' +
                  str(num) + '.world.xacro')
    print 'All ', len(coordinates), ' worlds generated'


def world_gen(coordinate={}, master={}):
    world = {}
    for axis_name, axis in master.iteritems():

        # if a sequence override defined for this axis at this step
        if axis['sequence'] is not None and \
                coordinate[axis_name] in axis['sequence']:
            # instances of macros already present in the world dict
            for i in world:
                if i in axis['sequence'][coordinate[axis_name]]:
                    world[i] += axis['sequence'][coordinate[axis_name]]
            # add all unique macros in sequence to world
            for i in axis['sequence'][coordinate[axis_name]]:
                if i not in world:
                    if axis['sequence'][coordinate[axis_name]][i] == [None]:
                        world[i] = [{}]
                    else:
                        world[i] = axis['sequence'][coordinate[axis_name]][i]
        # for the non-sequence override case:
        else:
            for macro_name, macro_calls in axis['macros'].iteritems():
                # if this one is new
                if macro_name not in world:
                    world[macro_name] = []
                for params in macro_calls:
                    if params is not None:
                        evaluated_params = {}
                        for param, value in params.iteritems():
                            # values flanked by ' evaluated as strings
                            if value[0] == "'" and value[-1] == "'":
                                evaluated_params[param] = value[1:-1]
                            # values not flanked by ' evaluated as lambdas
                            else:
                                f = str(value)
                                evaluated_params[param] =\
                                    (lambda n: eval(f))(coordinate[axis_name])
                        world[macro_name].append(evaluated_params)
                    else:
                        world[macro_name].append({})
    return world


def linear_combinations(master={}):
    combinations = []
    axies = OrderedDict()
    start = {}
    # make the starting spot and max
    # NOTE:the start coordinate <= all coordinates on an axis <= axies max
    for axis, value in master.iteritems():
        start[axis] = 0
        axies[axis] = value['steps']-1
    iterate(axies_max=axies, coordinates=combinations,
            current_coordinate=start)
    return combinations


def iterate(axies_max={}, coordinates=[], current_coordinate={}):
    coordinates.append(current_coordinate.copy())
    # if we are at the max coordinate, return
    if axies_max == current_coordinate:
        return
    else:
        for axis in current_coordinate:
            if current_coordinate[axis] < axies_max[axis]:
                current_coordinate[axis] += 1
                break
            else:
                current_coordinate[axis] = 0
        iterate(axies_max=axies_max, coordinates=coordinates,
                current_coordinate=current_coordinate)
