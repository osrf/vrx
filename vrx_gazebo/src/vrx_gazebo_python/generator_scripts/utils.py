import os
import yaml


def create_xacro_file(xacro_target,
                    yaml_file=None,
                    requested_macros={},
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
    if requested_macros == {}:
        s = open(yaml_file, 'r')
        requested_macros = yaml.load(s)

        # Handle case with empty yaml file 
        if requested_macros is None:
            xacro_file.write(boiler_plate_bot)
            xacro_file.close()
            return

    for key, objects in requested_macros.items():
        # object must be available
        # can only have so many of this type of object
        assert num_test(key, len(objects)), \
            "%d %s's not allowed" % (len(objects), key)
        xacro_file.write('  <!-- === %s === -->\n' % key)
        for i in objects:
            # test the parameter list and make sure it is in accordance
            assert param_test(key, i), \
                "%s %s failed parameter test" % (key, i['name'])
            xacro_file.write(macro_call_gen(key, i))

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
    macro_call = '  <xacro:%s ' % name
    for i in params:
        macro_call += '%s="%s" ' % (i, str(params[i]))
    macro_call += '/>\n'
    return macro_call


def get_macros(directory):
    xacro_files = get_macro_files(directory)
    macros = {}
    for i in xacro_files:
        name, params = parse_xacro_file(i)
        macros[name] = params
    return macros


def get_macro_files(directory):
    xacro_files = [directory+'/'+f for f in os.listdir(directory)
                   if os.path.isfile(os.path.join(directory, f)) and
                   (f[-6:] == '.xacro')]
    child_directories = [d[0] for d in os.walk(directory)]
    child_directories = child_directories[1:]
    for i in child_directories:
        for j in get_macro_files(i):
            xacro_files.append(j)
    return xacro_files


def parse_xacro_file(xacro_file_name):
    xacro_file = open(xacro_file_name, 'r')
    contents = xacro_file.read()
    # remove comment blocks
    while '<!--' in contents:
        start = contents.find('<!--')
        end = contents.find('-->')
        contents = contents.replace(contents[start:end+3], '')
    # get macro declaration
    start = contents.find('<xacro:macro')
    end = contents.find('>', start)
    declaration = contents[start:end]

    contents = contents.replace(contents[start:end+1], '')
    # remove the declaration from the string so we know we processed it
    name_pose = declaration.find('name')
    start = declaration.find('"', name_pose)
    end = declaration.find('"', start+1)
    name = declaration[start+1:end]

    params_pose = declaration.find('params')
    start = declaration.find('"', params_pose)
    end = declaration.find('"', start+1)
    params_str = declaration[start+1:end].split(' ')
    params = {}
    for i in params_str:
        i = i.replace('\n', '')
        if i != '':
            key = i
            if ':' in i:
                key = i[:i.find(':')]
            if '=' in i:
                value = i[i.find('=')+1:]
            else:
                value = ''
            value = value.replace('\n', '')
            params[key] = value

    if contents.find('<xacro:macro') != -1:
        raise Exception('multiple macros defined in %s' % xacro_file_name)

    return name, params
