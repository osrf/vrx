import os
import yaml


def create_xacro_file(xacro_target,
                    yaml_file=None,
                    requested_macros=None,
                    boiler_plate_top='',
                    boiler_plate_bot='',
                    num_test=lambda name, num: True,
                    param_test=lambda name, params={}: True,
                    xacro_type="",
                    ):
    """
    Purpose: Create a .xacro file for the purpose of creating a custom WAM-V .urdf

    Args:
        xacro_target (str): Target file for writing the xacro to 
                            NOTE: will overwrite an existing file
        yaml_file (str): .yaml file with requested macros
        requested_macros (dict): if dictionary is passed directly, no yaml file needed
        boiler_plate_top (str): String to start the xacro file
        boiler_plate_bot (str): String to end the xacro file
        num_test (function): test if the number of macro types is allowed
        param_test (function): test if a given macro call parameters are sensible
        xacro_type (str): type of xacro file

    Creates a xacro file at 'xacro_target'
    """
    # Initialize xacro file
    xacro_file = open(xacro_target, 'wb')
    xacro_file.write(boiler_plate_top)

    # If requested_macros not given, then open yaml_file
    if requested_macros is None:
        s = open(yaml_file, 'r')
        requested_macros = yaml.load(s)

        # Handle case with empty yaml file 
        if requested_macros is None:
            xacro_file.write(boiler_plate_bot)
            xacro_file.close()
            return

    # Object must be available
    for key, objects in requested_macros.items():
        # Check if number of objects is valid
        assert num_test(key, len(objects)), \
            "%d %s's not allowed" % (len(objects), key)

        # Create block for each object
        xacro_file.write('    <!-- === %s === -->\n' % key)
        for i in objects:
            # Check for valid parameters
            assert param_test(key, i), \
                "%s %s failed parameter test" % (key, i['name'])

            # Write macro
            xacro_file.write('    ' + macro_call_gen(key, i))
        xacro_file.write('\n')

    if xacro_type == "thruster":
        # WAM-V Gazebo thrust plugin setup
        xacro_file.write('  <gazebo>\n')
        xacro_file.write('    <plugin name="wamv_gazebo_thrust" filename="libusv_gazebo_thrust_plugin.so">\n')
        xacro_file.write('      <cmdTimeout>1.0</cmdTimeout>\n')
        xacro_file.write('      ' + macro_call_gen('include', {'filename': '$(find wamv_gazebo)/urdf/thruster_layouts/wamv_gazebo_thruster_config.xacro'}))
        for key, objects in requested_macros.items():
            for obj in objects:
                xacro_file.write('      ' + macro_call_gen('wamv_gazebo_thruster_config', {'name': obj['prefix']}))
        xacro_file.write('    </plugin>\n')
        xacro_file.write('  </gazebo>\n')

    xacro_file.write(boiler_plate_bot)
    xacro_file.close()


def macro_call_gen(name, params={}):
    macro_call = '<xacro:%s ' % name
    for i in params:
        macro_call += '%s="%s" ' % (i, str(params[i]))
    macro_call += '/>\n'
    return macro_call

