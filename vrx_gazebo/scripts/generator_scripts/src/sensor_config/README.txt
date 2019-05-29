Quick Start Instructions:
	Step 1:
		run the script:
			roslaunch vrx_gazebo generate_sensors.launch 
	Step 2:
		see the confirmation message "sensors sucessfully genreated" in the terminal and close the script (CTRL+c)
	Step 3:
		launch the example world:
			roslaunch vrx_gazebo sandisland.launch
	Step 4:
		look at the WAM-V in Sand island. It currently has the default sensor configuration.
	Step 5:
		close vrx
			Ctrl+c
	Step 6:
		create your own custom sensor configuration yaml (you can use vrx/vrx_gazebo/scripts/generator_scripts/src/sensor_config/sensor_config.yaml as a template)
	Step 7:
		re-launch the script with your custom yaml file(close script once confirmation message):
			roslaunch vrx_gazebo generate_sensors.launch available:=/root/path/to/your/file.yaml
	Step 8:
		relaunch sandisland (Step 3)
	Step 9:
		confirm that these are the sensors you want in the places that you want
	Step 10:
		observe the ros topics being published and confirm they are the topics you want:
			rostopic list
Description:
	generate_sensors.py is a simple script that allows sensor configurations to be submitted by YAML (as oppossed to urdf) while making sure that the sensors are in compliance as defined by sensor_config/compliance.py and are allowed (defined in ~/vrx_ws/src/vrx/wamv_gazebo/urdf/sensors).
	
	It operates by looking at all macros defined in a directory(in this case, ~/vrx_ws/src/vrx/wamv_gazebo/urdf/sensors) and all macros called in sensor_config.yaml (as well as their parameters). It makes sure that all the macros called are avalible in ~/vrx_ws/src/vrx/wamv_gazebo/urdf/sensors as well as the specified parameters. It also checks the number of times a macro is called for compliance ie: only ONE lidar allowed. It also checks the parameters of each macro callfor compliance ie: all sensors must be in a bounding box around the WAM-V.

	If the sensor configuration passes, then the script auto fills out ~/vrx_ws/src/vrx/wamv_description/urdf/sensors.xacro

	NOTE:sensors.xacro CANNOT be empty, you will recive an XML error
