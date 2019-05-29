import os
import yaml
def macro_block_gen(target,#target file for writing the macro calls too NOTE: will write over if a file is already there
                    availible,#directory of macro/xacro files NOTE: will only examine files that end in .xacro NOTE: WILL search sub-directories
                    requested = None,#yaml file with requested macros
                    requested_macros = {},#for if a dictionary is passed directly, no yaml file needed
                    boiler_plate_top = '',#stuff to start the xacro file
                    boiler_plate_bot = '',#stuff to end the xacro file
                    num_test = lambda a:True,#test if the number of a type of requested macros is allowed
                    param_test = lambda a:True,#test if a given macro call parameters are sensable(NOT if the parameters are presentfor a given macro)
                    var = lambda name, params={}: params):#add variance to a given set of full params for a type of macro 
    xacro_file=open(target, 'wb')
    xacro_file.write(boiler_plate_top)

    availible_macros = get_macros(availible)
    print availible_macros
    if requested_macros == {}:
        s = open(requested,'r')
        requested_macros = yaml.load(s)

    for key, objects in requested_macros.items():
        #object must be availible
        assert key in availible_macros.keys(),"%s is not defined in %s"%(key, availible)
        #can only have so many of this type of object
        assert num_test(key, len(objects)), "%d %s's are not allowed"%(len(objects),key)
        xacro_file.write('  <!-- === %s === -->\n'% (key))
        for i in objects:
            full_params = availible_macros[key].copy()#dict of default params for this object
            if i != None:
                for j in i:
                    #all params in all objects must be correct
                    assert j in availible_macros[key].keys(),"%s is not a parameter in %s"%(j,key)
                    full_params[j] = i[j]#replace the specified param's value = the value specified in yaml file
                full_params = var(key, full_params)
            #test the full parameter list and make sure it is in accordance
            assert param_test(key, full_params),"%s %s failed parameter test"%(key, i['name'])
            xacro_file.write(macro_call_gen(key, full_params))
    xacro_file.write(boiler_plate_bot)
    xacro_file.close()

def macro_call_gen(name, params={}):
    macro_call='  <xacro:%s '%name
    for i in params:
        macro_call+='%s="%s" '%(i, str(params[i]))
    macro_call+='/>\n'
    return macro_call

def get_macros(directory):
    xacro_files = get_macro_files(directory)
    macros = {}
    for i in xacro_files:
        name, params = parse_xacro_file(i)
        macros[name] = params
    return macros

def get_macro_files(directory):
    xacro_files = [directory+'/'+f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory,f)) and (f[-6:] == '.xacro')]
    child_directories = [d[0] for d in os.walk(directory)]
    child_directories = child_directories[1:]
    
    for i in child_directories:
        for j in get_macro_files(i):
            xacro_files.append(j)
    
    return xacro_files

def parse_xacro_file(xacro_file_name):
    xacro_file=open(xacro_file_name, 'r')
    contents=xacro_file.read()
    #remove comment blocks
    while '<!--' in contents:
        start=contents.find('<!--')
        end = contents.find('-->')
        contents = contents.replace(contents[start:end+3], '')
    #get macro declaration
    start = contents.find('<xacro:macro')
    end = contents.find('>', start)
    declaration = contents[start:end]

    contents = contents.replace(contents[start:end+1],'') #remove the declaration from the string so we know we processed it
    name_pose = declaration.find('name')
    start = declaration.find('"', name_pose)
    end = declaration.find('"',start+1)
    name = declaration[start+1:end]

    params_pose =declaration.find('params')
    start = declaration.find('"', params_pose)
    end = declaration.find('"',start+1)
    params_str =declaration[start+1:end].split(' ')
    params={}
    for i in params_str:
        if i != '':
            key = i
            if ':' in i:
                key = i[:i.find(':')]
            if '=' in i:
                value = i[i.find('=')+1:]
            else:
                value = ''
            value = value.replace('\n', '')
            params[key]=value

    if contents.find('<xacro:macro') != -1:
        raise Exception('multiple macros defined in %s'%xacro_file_name)

    return name, params
