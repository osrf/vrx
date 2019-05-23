import os
import yaml
def macro_block_gen(availible, requested, target, boiler_plate, num_test, param_test):
    xacro_file=open(target, 'wb')
    xacro_file.write(boiler_plate)

    availible_macros = get_macros(availible)

    s = open(requested,'r')
    requested_macros = yaml.load(s)

    for key, objects in requested_macros.items():
        #object must be availible
        assert key in availible_macros.keys(),"%s is not defined in %s"%(key, availible)
        #can only have so many of this type of object
        assert num_test(key, len(objects)), "%d %s's are not allowed"%(len(objects),key)
        xacro_file.write('  <!-- === %s === -->\n'% (key))
        for i in objects:
            full_params = availible_macros[key].params.copy()#dict of default params for this object
            for j in i:
                #all params in all objects must be correct
                assert j in availible_macros[key].params.keys(),"%s is not a parameter in %s"%(j,key)
                full_params[j] = i[j]#replace the specified param's value = the value specified in yaml file
            #test the full parameter list and make sure it is in accordance
            assert param_test(key, full_params),"%s %s failed parameter test"%(key, i['name'])
            xacro_file.write(macro_call_gen(key, i))
    xacro_file.write('</xacro:macro>\n')
    xacro_file.write('</robot>\n')
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
        macro = Macro(i)
        macros[macro.name] = macro
    return macros

def get_macro_files(directory):
    xacro_files = [directory+'/'+f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory,f)) and (f[-6:] == '.xacro')]
    child_directories = [d[0] for d in os.walk(directory)]
    child_directories = child_directories[1:]
    
    for i in child_directories:
        for j in get_macro_files(i):
            xacro_files.append(j)
    
    return xacro_files

class Macro:
    def __init__(self, xacro_file_name):
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
            key = i
            if ':' in i:
                key = i[:i.find(':')]
            if '=' in i:
                value = i[i.find('=')+1:]
            else:
                value = ''
            params[key]=value
        
        
        self.name = name
        self.params = params
       
        if contents.find('<xacro:macro') != -1:
            raise Exception('multiple macros defined in %s'%xacro_file_name)


