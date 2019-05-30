Quick Start Instructions:
	Step 1:
		launch the example world:
			roslaunch vrx_gazebo sandisland.launch
	Step 2:
		look at the WAM-V in sand island. It currently has the default sensor configuration.
	Step 3:
		close gazebo
			Ctrl+c
	Step 4:
		make a directory for your wamv:
			mkdir ~/my_wamv
			cd ~/my_wamv
	Step 5:
		make a yaml file of the sensors you want and where you want them (see example yaml sensor configuration file):
			gedit sensor_config.yaml
	Step 6:
		run the script to generate your wamv's urdf with these sensors:
			 roslaunch vrx_gazebo generate_sensors.launch requested:=/home/<username>/my_wamv/sensor_config.yaml xacro_target:=/home/<username>/my_wamv/my_sensors.xacro wamv_target:=/home/<username>/my_wamv/my_wamv.urdf

		requested explained:
			this is the root path of your yaml sensor configuration file
		xacro_target explained:
			this is the root path of a xacro which will be generated based on your yaml file, it needs a place to live.
		wamv_target explained:
			this is the root path to the urdf of your wamv which will be generated
	Step 7:
		see the confirmation message "wamv sucessfully generated" in the terminal and close the script (CTRL+c)
	Step 8:
		launch the example world with your wamv:
			roslaunch vrx_gazebo sandisland.launch urdf:=/home/<username>/my_wamv/my_wamv.urdf
	Step 9:
		look at the WAM-V in Sand island. It has your sensor configuration
	Step 10:
		confirm that these are the sensors you want in the places that you want
	Step 11:
		observe the ros topics being published and confirm they are the topics you want:
			rostopic list
	Step 12:
		close gazebo:
			Ctrl+c
Description:
	generate_sensors.launch is a simple script that allows sensor configurations to be submitted by YAML (as oppossed to urdf) while making sure that the sensors are in compliance as defined by sensor_config/compliance.py and are allowed (defined in wamv_gazebo/urdf/sensors).
	
	It operates by looking at all macros defined in a directory(in this case, wamv_gazebo/urdf/sensors) and all macros called (as well as their parameters). It makes sure that all the macros called are avalible in wamv_gazebo/urdf/sensors as well as the specified parameters. It also checks the number of times a macro is called for compliance ie: only one lidar allowed. It also checks the parameters of each macro call for compliance ie: all sensors must be in a bounding box around the WAM-V.

	If the sensor configuration passes, the script auto fills out a xacro at xacro_target and calls a xacro command to generate the urdf at wamv_target using wamv_gazebo/urdf/wamv_gazebo.urdf.xacro.

example yaml sensor configuration file:

wamv_camera:
    - name: front_camera
      x: 0.75
      y: 0.3
      z: 2
      P: ${radians(15)}
    - name: front_left_camera
      x: 0.75
      y: 0.1
      P: ${radians(15)}
    - name: front_right_camera
      x: 0.75
      y: -0.1
      P: ${radians(15)}
    - name: middle_right_camera
      y: 0.1
      x: 0.75
      P: ${radians(15)}
wamv_gps:
    - name: gps_wamv
      x: -0.85
wamv_imu:
    - name: imu_wamv
      y: -0.2
wamv_p3d:
    - name: p3d_wamv
wamv_3d_lidar:
    - name: lidar_wamv
      P: ${radians(8)}
