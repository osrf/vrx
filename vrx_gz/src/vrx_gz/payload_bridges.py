from vrx_gz.bridge import Bridge, BridgeDirection

import sdformat13 as sdf


def gz_prefix(world_name, model_name, link_name, sensor_name):
    return f'/world/{world_name}/model/{model_name}/link/{link_name}/sensor/{sensor_name}'


def ros_prefix(sensor_name, sensor_type):
    return f'sensors/{sensor_type}/{sensor_name}'


def image(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'cameras')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/image',
        ros_topic=f'{ros_sensor_prefix}/image_raw',
        gz_type='ignition.msgs.Image',
        ros_type='sensor_msgs/msg/Image',
        direction=BridgeDirection.GZ_TO_ROS)


def depth_image(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'cameras')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/depth_image',
        ros_topic=f'{ros_sensor_prefix}/depth',
        gz_type='ignition.msgs.Image',
        ros_type='sensor_msgs/msg/Image',
        direction=BridgeDirection.GZ_TO_ROS)


def camera_info(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'cameras')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/camera_info',
        ros_topic=f'{ros_sensor_prefix}/camera_info',
        gz_type='ignition.msgs.CameraInfo',
        ros_type='sensor_msgs/msg/CameraInfo',
        direction=BridgeDirection.GZ_TO_ROS)


def lidar_scan(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'lidars')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/scan',
        ros_topic=f'{ros_sensor_prefix}/scan',
        gz_type='ignition.msgs.LaserScan',
        ros_type='sensor_msgs/msg/LaserScan',
        direction=BridgeDirection.GZ_TO_ROS)


def lidar_points(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'lidars')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/scan/points',
        ros_topic=f'{ros_sensor_prefix}/points',
        gz_type='ignition.msgs.PointCloudPacked',
        ros_type='sensor_msgs/msg/PointCloud2',
        direction=BridgeDirection.GZ_TO_ROS)


def camera_points(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'cameras')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/points',
        ros_topic=f'{ros_sensor_prefix}/points',
        gz_type='ignition.msgs.PointCloudPacked',
        ros_type='sensor_msgs/msg/PointCloud2',
        direction=BridgeDirection.GZ_TO_ROS)


def rfranger(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name)
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/rfsensor',
        ros_topic=f'{ros_sensor_prefix}/rfsensor',
        gz_type='ignition.msgs.Param_V',
        ros_type='ros_gz_interfaces/msg/ParamVec',
        direction=BridgeDirection.GZ_TO_ROS)

def imu(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix('', 'imu')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/imu',
        ros_topic=f'{ros_sensor_prefix}imu/data',
        gz_type='ignition.msgs.IMU',
        ros_type='sensor_msgs/msg/Imu',
        direction=BridgeDirection.GZ_TO_ROS)

def navsat(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix('', 'gps')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/navsat',
        ros_topic=f'{ros_sensor_prefix}gps/fix',
        gz_type='ignition.msgs.NavSat',
        ros_type='sensor_msgs/msg/NavSatFix',
        direction=BridgeDirection.GZ_TO_ROS)


def contacts():
    return Bridge(
        gz_topic=f'/vrx/contacts',
        ros_topic=f'/vrx/contacts',
        gz_type='ignition.msgs.Contacts',
        ros_type='ros_gz_interfaces/msg/Contacts',
        direction=BridgeDirection.GZ_TO_ROS)


def odometry(model_name):
    ros_sensor_prefix = ros_prefix('', 'position')
    return Bridge(
        gz_topic=f'/model/{model_name}/odometry',
        ros_topic=f'{ros_sensor_prefix}ground_truth_odometry',
        gz_type='ignition.msgs.Odometry',
        ros_type='nav_msgs/msg/Odometry',
        direction=BridgeDirection.GZ_TO_ROS)

def ball_shooter(model_name):
    return Bridge(
        gz_topic=f'/{model_name}/shooters/ball_shooter/fire',
        ros_topic=f'shooters/ball_shooter/fire',
        gz_type='ignition.msgs.Boolean',
        ros_type='std_msgs/msg/Bool',
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
        gz_topic=f'{model_name}/sensors/acoustics/receiver/range_bearing',
        ros_topic=f'sensors/acoustics/receiver/range_bearing',
        gz_type='ignition.msgs.Param',
        ros_type='ros_gz_interfaces/msg/ParamVec',
        direction=BridgeDirection.GZ_TO_ROS)

def set_acoustic_pinger():
    return Bridge(
        gz_topic=f'/pinger/set_pinger_position',
        ros_topic=f'/pinger/set_pinger_position',
        gz_type='ignition.msgs.Vector3d',
        ros_type='geometry_msgs/msg/Vector3',
        direction=BridgeDirection.ROS_TO_GZ)

def payload_bridges(world_name, model_name, link_name, sensor_name, sensor_type):
    bridges = []
    if sensor_type == sdf.Sensortype.CAMERA:
        bridges = [
            image(world_name, model_name, link_name, sensor_name),
            camera_info(world_name, model_name, link_name, sensor_name)
        ]
    elif sensor_type == sdf.Sensortype.IMU:
        bridges = [
            imu(world_name, model_name, link_name, sensor_name)
        ]
    elif sensor_type == sdf.Sensortype.CONTACT:
        bridges = [
            contacts(),
        ]
    elif sensor_type == sdf.Sensortype.NAVSAT:
        bridges = [
            navsat(world_name, model_name, link_name, sensor_name),
        ]
    elif sensor_type == sdf.Sensortype.GPU_LIDAR:
        bridges = [
            lidar_scan(world_name, model_name, link_name, sensor_name),
            lidar_points(world_name, model_name, link_name, sensor_name)
        ]
    elif sensor_type == sdf.Sensortype.RGBD_CAMERA:
        bridges = [
            image(world_name, model_name, link_name, sensor_name),
            camera_info(world_name, model_name, link_name, sensor_name),
            depth_image(world_name, model_name, link_name, sensor_name),
            camera_points(world_name, model_name, link_name, sensor_name),
        ]
    elif 'OdometryPublisher' in sensor_name:
        bridges = [
            odometry(model_name),
        ]
    elif 'BallShooter' in sensor_name:
        bridges = [
            ball_shooter(model_name),
        ]
    elif 'thruster_thrust_' in sensor_name:
        bridges = [
            thrust(model_name, sensor_type),
        ]
    elif 'thruster_rotate_' in sensor_name:
        bridges = [
            thrust_joint_pos(model_name, sensor_type),
        ]
    elif 'AcousticPinger' in sensor_name:
        bridges = [
            acoustic_pinger(model_name),
            set_acoustic_pinger(),
        ]

    return bridges
