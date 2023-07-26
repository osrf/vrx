from vrx_gz.bridge import Bridge, BridgeDirection


def prefix(world_name, model_name, link_name):
    return f'/world/{world_name}/model/{model_name}/link/{link_name}/sensor'

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

def joint_states(world_name, model_name):
    return Bridge(
        gz_topic=f'/world/{world_name}/model/{model_name}/joint_state',
        ros_topic='joint_states',
        gz_type='ignition.msgs.Model',
        ros_type='sensor_msgs/msg/JointState',
        direction=BridgeDirection.GZ_TO_ROS)

def cmd_vel(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/cmd_vel',
        ros_topic='cmd_vel',
        gz_type='ignition.msgs.Twist',
        ros_type='geometry_msgs/msg/Twist',
        direction=BridgeDirection.ROS_TO_GZ)

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

def acoustic_tracking_mean_pose_error():
    return Bridge(
        gz_topic=f'/vrx/acoustic_tracking/mean_pose_error',
        ros_topic=f'/vrx/acoustic_tracking/mean_pose_error',
        gz_type='ignition.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)

def acoustic_tracking_pose_error():
    return Bridge(
        gz_topic=f'/vrx/acoustic_tracking/pose_error',
        ros_topic=f'/vrx/acoustic_tracking/pose_error',
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

def animal_pose(topic):
    return Bridge(
        gz_topic=f'{topic}',
        ros_topic=f'{topic}',
        gz_type='ignition.msgs.Pose',
        ros_type='geometry_msgs/msg/PoseStamped',
        direction=BridgeDirection.GZ_TO_ROS)

def gymkhana_blackbox_goal():
    return Bridge(
        gz_topic=f'/vrx/gymkhana_blackbox/goal',
        ros_topic=f'/vrx/gymkhana_blackbox/goal',
        gz_type='ignition.msgs.Pose',
        ros_type='geometry_msgs/msg/PoseStamped',
        direction=BridgeDirection.GZ_TO_ROS)

def gymkhana_blackbox_mean_pose_error():
    return Bridge(
        gz_topic=f'/vrx/gymkhana_blackbox/mean_pose_error',
        ros_topic=f'/vrx/gymkhana_blackbox/mean_pose_error',
        gz_type='ignition.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)

def gymkhana_blackbox_pose_error():
    return Bridge(
        gz_topic=f'/vrx/gymkhana_blackbox/pose_error',
        ros_topic=f'/vrx/gymkhana_blackbox/pose_error',
        gz_type='ignition.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)

def color_sequence_reports():
    return Bridge(
        gz_topic=f'/vrx/scan_dock_deliver/color_sequence',
        ros_topic=f'/vrx/scan_dock_deliver/color_sequence',
        gz_type='ignition.msgs.StringMsg_V',
        ros_type='ros_gz_interfaces/msg/StringVec',
        direction=BridgeDirection.ROS_TO_GZ)

def usv_wind_speed():
    return Bridge(
        gz_topic=f'/vrx/debug/wind/speed',
        ros_topic=f'/vrx/debug/wind/speed',
        gz_type='ignition.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)
        
def usv_wind_direction():
    return Bridge(
        gz_topic=f'/vrx/debug/wind/direction',
        ros_topic=f'/vrx/debug/wind/direction',
        gz_type='ignition.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)