from vrx_gz.bridge import Bridge, BridgeDirection


def prefix(world_name, model_name, link_name):
    return f'/world/{world_name}/model/{model_name}/link/{link_name}/sensor'


def imu(world_name, model_name, link_name='base_link'):
    sensor_prefix = prefix(world_name, model_name, link_name)
    return Bridge(
        gz_topic=f'{sensor_prefix}/imu_sensor/imu',
        ros_topic='imu/data',
        gz_type='ignition.msgs.IMU',
        ros_type='sensor_msgs/msg/Imu',
        direction=BridgeDirection.GZ_TO_ROS)


def magnetometer(world_name, model_name, link_name='base_link'):
    sensor_prefix = prefix(world_name, model_name, link_name)
    return Bridge(
        gz_topic=f'{sensor_prefix}/magnetometer/magnetometer',
        ros_topic='magnetic_field',
        gz_type='ignition.msgs.Magnetometer',
        ros_type='sensor_msgs/msg/MagneticField',
        direction=BridgeDirection.GZ_TO_ROS)


def air_pressure(world_name, model_name, link_name='base_link'):
    sensor_prefix = prefix(world_name, model_name, link_name)
    return Bridge(
        gz_topic=f'{sensor_prefix}/air_pressure/air_pressure',
        ros_topic='air_pressure',
        gz_type='ignition.msgs.FluidPressure',
        ros_type='sensor_msgs/msg/FluidPressure',
        direction=BridgeDirection.GZ_TO_ROS)

def pose(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/pose',
        ros_topic='pose',
        gz_type='ignition.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.GZ_TO_ROS)


def pose_static(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/pose_static',
        ros_topic='pose_static',
        gz_type='ignition.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.GZ_TO_ROS)


def cmd_vel(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/cmd_vel',
        ros_topic='cmd_vel',
        gz_type='ignition.msgs.Twist',
        ros_type='geometry_msgs/msg/Twist',
        direction=BridgeDirection.ROS_TO_GZ)

def thrust(model_name, side):
    return Bridge(
        gz_topic=f'{model_name}/thrusters/{side}/thrust',
        ros_topic=f'thrusters/{side}/thrust',
        gz_type='ignition.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_GZ)


def thrust_joint_pos(model_name, side):
    # ROS naming policy indicates that first character of a name must be an alpha
    # character. In the case below, the gz topic has the joint index 0 as the
    # first char so the following topics fail to be created on the ROS end
    # left_joint_topic = '/model/' + model_name + '/joint/left_chasis_engine_joint/0/cmd_pos'
    # right_joint_topic = '/model/' + model_name + '/joint/right_chasis_engine_joint/0/cmd_pos'
    # For now, use erb to generate unique topic names in model.sdf.erb
    return Bridge(
        gz_topic=f'{model_name}/thrusters/{side}/pos',
        ros_topic=f'thrusters/{side}/pos',
        gz_type='ignition.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_GZ)

def acoustic_pinger(model_name):
    return Bridge(
        gz_topic=f'{model_name}/pingers/pinger/range_bearing',
        ros_topic=f'pingers/pinger/range_bearing',
        gz_type='ignition.msgs.Param',
        ros_type='ros_gz_interfaces/msg/ParamVec',
        direction=BridgeDirection.GZ_TO_ROS)

def comms_tx(model_name):
    return Bridge(
        gz_topic='/broker/msgs',
        ros_topic='tx',
        gz_type='ignition.msgs.Dataframe',
        ros_type='ros_gz_interfaces/msg/Dataframe',
        direction=BridgeDirection.ROS_TO_GZ)


def comms_rx(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/rx',
        ros_topic='rx',
        gz_type='ignition.msgs.Dataframe',
        ros_type='ros_gz_interfaces/msg/Dataframe',
        direction=BridgeDirection.GZ_TO_ROS)


def clock():
    return Bridge(
        gz_topic='/clock',
        ros_topic='/clock',
        gz_type='ignition.msgs.Clock',
        ros_type='rosgraph_msgs/msg/Clock',
        direction=BridgeDirection.GZ_TO_ROS)


def task_info():
    return Bridge(
        gz_topic=f'/vrx/task/info',
        ros_topic=f'/vrx/task/info',
        gz_type='ignition.msgs.Param',
        ros_type='ros_gz_interfaces/msg/ParamVec',
        direction=BridgeDirection.GZ_TO_ROS)


def contacts():
    return Bridge(
        gz_topic=f'/vrx/contacts',
        ros_topic=f'/vrx/contacts',
        gz_type='ignition.msgs.Contacts',
        ros_type='ros_gz_interfaces/msg/Contacts',
        direction=BridgeDirection.GZ_TO_ROS)


def stationkeeping_goal():
    return Bridge(
        gz_topic=f'/vrx/stationkeeping/goal',
        ros_topic=f'/vrx/stationkeeping/goal',
        gz_type='ignition.msgs.Pose',
        ros_type='geometry_msgs/msg/PoseStamped',
        direction=BridgeDirection.GZ_TO_ROS)


def stationkeeping_mean_pose_error():
    return Bridge(
        gz_topic=f'/vrx/stationkeeping/mean_pose_error',
        ros_topic=f'/vrx/stationkeeping/mean_pose_error',
        gz_type='ignition.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)


def stationkeeping_pose_error():
    return Bridge(
        gz_topic=f'/vrx/stationkeeping/pose_error',
        ros_topic=f'/vrx/stationkeeping/pose_error',
        gz_type='ignition.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)


def wayfinding_waypoints():
    return Bridge(
        gz_topic=f'/vrx/wayfinding/waypoints',
        ros_topic=f'/vrx/wayfinding/waypoints',
        gz_type='ignition.msgs.Pose_V',
        ros_type='geometry_msgs/msg/PoseArray',
        direction=BridgeDirection.GZ_TO_ROS)


def wayfinding_mean_error():
    return Bridge(
        gz_topic=f'/vrx/wayfinding/mean_error',
        ros_topic=f'/vrx/wayfinding/mean_error',
        gz_type='ignition.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)


def wayfinding_min_errors():
    return Bridge(
        gz_topic=f'/vrx/wayfinding/min_errors',
        ros_topic=f'/vrx/wayfinding/min_errors',
        gz_type='ignition.msgs.Float_V',
        ros_type='ros_gz_interfaces/msg/Float32Array',
        direction=BridgeDirection.GZ_TO_ROS)

def perception_reports():
    return Bridge(
        gz_topic=f'/vrx/perception/landmark',
        ros_topic=f'/vrx/perception/landmark',
        gz_type='ignition.msgs.Pose',
        ros_type='geometry_msgs/msg/PoseStamped',
        direction=BridgeDirection.ROS_TO_GZ)
