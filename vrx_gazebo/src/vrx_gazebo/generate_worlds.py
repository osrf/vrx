#!/usr/bin/env python3
import yaml
import rospy
import os
from collections import OrderedDict

from vrx_gazebo.utils import create_xacro_file


def main():
    competition_pkg = rospy.get_param('competition_pkg')
    world_name = rospy.get_param('world_name')
    s = open(rospy.get_param('requested'))
    yaml_name = rospy.get_param('requested')[
        rospy.get_param('requested').rfind('/')+1:
        rospy.get_param('requested').rfind('.yaml')]

    # get the yaml as a dict
    master = yaml.safe_load(s)

    # get list of all coordinates that the master dict maps out
    coordinates = linear_combinations(master)

    # create a world xacro and subsequent world file for each coordinate
    for world_num, coord in enumerate(coordinates):

        world_file = rospy.get_param('world_target') + yaml_name + \
            str(world_num) + '.world'
        xacro_file = rospy.get_param('world_xacro_target') + yaml_name + \
            str(world_num) + '.world.xacro'

        # Only used for gymkhana task
        config_file = rospy.get_param('config_target') + yaml_name + \
            str(world_num) + '.yaml'

        xacro_top = make_xacro_top(coord, world_name)
        create_xacro_file(
            xacro_target=rospy.get_param('world_xacro_target') +
                yaml_name + str(world_num) + '.world.xacro',
            requested_macros=world_gen(coordinate=coord, master=master,
                config_file=config_file),
            boiler_plate_top=xacro_top,
            boiler_plate_bot='</world>\n</sdf>')

        # Convert xacro file to world file
        os.system('rosrun xacro xacro -o ' + world_file + ' ' + xacro_file)

    print('All %d worlds generated' % len(coordinates))

def make_xacro_top(coord, world_name):
  coord_comment  = '<!-- COORDINATE: ' + str(coord) + ' -->'
  world_name_el  = '<world name="' + world_name + '">'
  xacro_includes = """\
  <xacro:include filename="$(find vrx_2019)/worlds/sandisland.xacro" />
  <xacro:include filename="$(find vrx_gazebo)/worlds/xacros/sandisland_minus_scene.xacro" />
  <xacro:include filename="$(find vrx_gazebo)/worlds/xacros/sydneyregatta_minus_scene.xacro" />
  <xacro:include filename="$(find vrx_gazebo)/worlds/xacros/scene.xacro" />
  <xacro:include filename="$(find vrx_gazebo)/worlds/xacros/nav_challenge.xacro" />
  <xacro:include filename="$(find vrx_gazebo)/worlds/xacros/usv_wind_plugin.xacro" />
  <xacro:include filename="$(find wave_gazebo)/world_models/ocean_waves/model.xacro"/>
  <xacro:include filename="$(find vrx_gazebo)/worlds/xacros/perception.xacro" />
  <xacro:include filename="$(find vrx_gazebo)/worlds/xacros/stationkeeping.xacro" />
  <xacro:include filename="$(find vrx_gazebo)/worlds/xacros/dock.xacro" />
  <xacro:include filename="$(find vrx_gazebo)/worlds/xacros/scan_and_dock.xacro" />
  <xacro:include filename="$(find vrx_gazebo)/worlds/xacros/wayfinding.xacro" />
  <xacro:include filename="$(find vrx_gazebo)/worlds/xacros/wildlife.xacro" />
  <xacro:include filename="$(find vrx_gazebo)/worlds/xacros/scan_dock_deliver.xacro" />
  <xacro:include filename="$(find vrx_gazebo)/worlds/xacros/gymkhana.xacro" />
  <xacro:include filename="$(find vrx_gazebo)/worlds/xacros/insert_model.xacro" />
  """

  xacro_top = '<?xml version="1.0" ?>\n'\
              '<sdf version="1.6" xmlns:xacro="http://ros.org/wiki/xacro">\n'

  xacro_top += coord_comment + '\n'
  xacro_top += world_name_el + '\n'
  xacro_top += xacro_includes + '\n'
  return xacro_top

def world_gen(coordinate={}, master={}, config_file=None):
    world = {}
    # axis_name: root-level key
    # axis: sub-tree from root-level key
    for axis_name, axis in iter(master.items()):

        # if a sequence override defined for this axis at this step
        # These are parameters to go into a xacro macro file
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
            for macro_name, macro_calls in iter(axis['macros'].items()):
                # if this one is new
                if macro_name not in world:
                    world[macro_name] = []
                for params in macro_calls:
                    if params is not None:
                        evaluated_params = {}
                        for param, value in iter(params.items()):
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

        # Only used for gymkhana task
        if 'yamls' in axis and \
                axis['yamls'] is not None and \
                coordinate[axis_name] in axis['yamls']:
            # Dump the subtree under this trial into a YAML file
            params = axis['yamls'][coordinate[axis_name]]
            config_stream = open(config_file, 'w')
            yaml.dump(params, config_stream)
            print("Generated %s" % config_file)

    return world


def linear_combinations(master={}):
    combinations = []
    axies = OrderedDict()
    start = {}
    # make the starting spot and max
    # NOTE: the start coordinate <= all coordinates on an axis <= axies max
    for axis, value in iter(master.items()):
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
