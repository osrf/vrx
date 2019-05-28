Quick Start Instructions:
	Step 1:
		navigate to the scripts directory:
			cd ~/vrx_ws/src/vrx/vrx_gazebo/scripts
	Step 2:
		run the generate_sensor.py script:
			python generate_sensors.py
	Step 3:
		launch the example world:
			roslaunch vrx_gazebo sandisland.launch
	Step 4:
		look at the WAM-V in Sand island. It currently has the default sensor configuration.
	Step 5:
		close vrx
			Ctrl+c
	Step 6:
		edit ~/vrx_ws/src/vrx/vrx_gazebo/scripts/generator_scripts/sensor_config/sensor_config.yaml such that is has the sensors you want in the configuration you want
			gedit ~/vrx_ws/src/vrx/vrx_gazebo/scripts/generator_scripts/sensor_config/sensor_config.yaml	
	Step 7:
		re-launch generate_sensors.py
	Step 8:
		relaunch sandisland (Step 3)
	Step 9:
		confirm that these are the sensors you want in the places that you want
	Step 10:
		observe the ros topics being published and confirm they are the topics you want
			rostopic list
Description:
	generate_sensors.py is a simple script that allows sensor configurations to be submitted by YAML (as oppossed to urdf) while making sure that the sensors are in compliance as defined by ~/vrx_ws/src/vrx/vrx_gazebo/scripts/generator_scripts/sensor_config/compliance.py and are allowed (defined in ~/vrx_ws/src/vrx/wamv_gazebo/urdf/sensors).
	
	It operates by looking at all macros defined in a directory(in this case, ~/vrx_ws/src/vrx/wamv_gazebo/urdf/sensors) and all macros called in sensor_config.yaml (as well as their parameters). It makes sure that all the macros called are avalible in ~/vrx_ws/src/vrx/wamv_gazebo/urdf/sensors as well as the specified parameters. It also checks the number of times a macro is called for compliance ie: only ONE lidar allowed. It also checks the parameters of each macro callfor compliance ie: all sensors must be in a bounding box around the WAM-V.

	If the sensor configuration passes, then the script auto fills out ~/vrx_ws/src/vrx/wamv_description/urdf/sensors.xacro.

	NOTE:sensors.xacro CANNOT be empty
