import re
import os

def get_macros(directory):
    xacro_files = [f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory,f))]
    macros = {}
    for i in xacro_files:
        macro = Macro(directory+str(i))
        macros[macro.name] = macro
    return macros

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
        name_pose = declaration.find('name')
        start = declaration.find('"', name_pose)
        end = declaration.find('"',start+1)
        name = declaration[start+1:end]

        params_pose =declaration.find('params')
        start = declaration.find('"', params_pose)
        end = declaration.find('"',start+1)
        params_raw =declaration[start+1:end].split(' ')
        params={}
        for i in params_raw:
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

