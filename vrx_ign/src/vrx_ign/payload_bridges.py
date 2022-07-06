from vrx_ign.bridge import Bridge, BridgeDirection


def camera_models():
    models = ['vrx_vga_camera',
              'vrx_hd_camera']
    return models


def rgbd_models():
    models = ['vrx_rgbd_camera']
    return models


def lidar_models():
    models = ['vrx_planar_lidar',
              'vrx_3d_lidar',
              ]
    return models


def rfranger_models():
    models = ['vrx_rf_range',
              'vrx_rf_long_range']
    return models


def slot_prefix(world_name, model_name, slot_idx, model_prefix=''):
    s = f'/world/{world_name}/model/{model_name}/model/'
    if model_prefix != '':
        s += f'{model_prefix}/model/'
    s += f'sensor_{slot_idx}/link/sensor_link/sensor'
    return s


def ros_slot_prefix(slot_idx, model_prefix=''):
    s = ''
    if model_prefix != '':
        s = f'{model_prefix}/'
    s += f'slot{slot_idx}'
    return s


def image(world_name, model_name, slot_idx, model_prefix=''):
    prefix = slot_prefix(world_name, model_name, slot_idx, model_prefix)
    ros_prefix = ros_slot_prefix(slot_idx, model_prefix)
    return Bridge(
        ign_topic=f'{prefix}/camera/image',
        ros_topic=f'{ros_prefix}/image_raw',
        ign_type='ignition.msgs.Image',
        ros_type='sensor_msgs/msg/Image',
        direction=BridgeDirection.IGN_TO_ROS)


def depth_image(world_name, model_name, slot_idx, model_prefix=''):
    prefix = slot_prefix(world_name, model_name, slot_idx, model_prefix)
    ros_prefix = ros_slot_prefix(slot_idx, model_prefix)
    return Bridge(
        ign_topic=f'{prefix}/camera/depth_image',
        ros_topic=f'{ros_prefix}/depth',
        ign_type='ignition.msgs.Image',
        ros_type='sensor_msgs/msg/Image',
        direction=BridgeDirection.IGN_TO_ROS)


def camera_info(world_name, model_name, slot_idx, model_prefix=''):
    prefix = slot_prefix(world_name, model_name, slot_idx, model_prefix)
    ros_prefix = ros_slot_prefix(slot_idx, model_prefix)
    return Bridge(
        ign_topic=f'{prefix}/camera/camera_info',
        ros_topic=f'{ros_prefix}/camera_info',
        ign_type='ignition.msgs.CameraInfo',
        ros_type='sensor_msgs/msg/CameraInfo',
        direction=BridgeDirection.IGN_TO_ROS)


def lidar_scan(world_name, model_name, slot_idx, model_prefix=''):
    prefix = slot_prefix(world_name, model_name, slot_idx, model_prefix)
    ros_prefix = ros_slot_prefix(slot_idx, model_prefix)
    return Bridge(
        ign_topic=f'{prefix}/lidar/scan',
        ros_topic=f'{ros_prefix}/scan',
        ign_type='ignition.msgs.LaserScan',
        ros_type='sensor_msgs/msg/LaserScan',
        direction=BridgeDirection.IGN_TO_ROS)


def lidar_points(world_name, model_name, slot_idx, model_prefix=''):
    prefix = slot_prefix(world_name, model_name, slot_idx, model_prefix)
    ros_prefix = ros_slot_prefix(slot_idx, model_prefix)
    return Bridge(
        ign_topic=f'{prefix}/lidar/scan/points',
        ros_topic=f'{ros_prefix}/points',
        ign_type='ignition.msgs.PointCloudPacked',
        ros_type='sensor_msgs/msg/PointCloud2',
        direction=BridgeDirection.IGN_TO_ROS)


def camera_points(world_name, model_name, slot_idx, model_prefix=''):
    prefix = slot_prefix(world_name, model_name, slot_idx, model_prefix)
    ros_prefix = ros_slot_prefix(slot_idx, model_prefix)
    return Bridge(
        ign_topic=f'{prefix}/camera/points',
        ros_topic=f'{ros_prefix}/points',
        ign_type='ignition.msgs.PointCloudPacked',
        ros_type='sensor_msgs/msg/PointCloud2',
        direction=BridgeDirection.IGN_TO_ROS)


def rfranger(world_name, model_name, slot_idx, model_prefix=''):
    prefix = f'/world/{world_name}/model/{model_name}/model/sensor_{slot_idx}'
    ros_prefix = ros_slot_prefix(slot_idx, model_prefix)
    return Bridge(
        ign_topic=f'{prefix}/rfsensor',
        ros_topic=f'{ros_prefix}/rfsensor',
        ign_type='ignition.msgs.Param_V',
        ros_type='ros_ign_interfaces/msg/ParamVec',
        direction=BridgeDirection.IGN_TO_ROS)


def payload_bridges(world_name, model_name, payload, idx, model_prefix=''):
    bridges = []
    if payload in camera_models():
        bridges = [
            image(world_name, model_name, idx, model_prefix),
            camera_info(world_name, model_name, idx, model_prefix)
        ]
    elif payload in lidar_models():
        bridges = [
            lidar_scan(world_name, model_name, idx, model_prefix),
            lidar_points(world_name, model_name, idx, model_prefix)
        ]
    elif payload in rgbd_models():
        bridges = [
            image(world_name, model_name, idx, model_prefix),
            camera_info(world_name, model_name, idx, model_prefix),
            depth_image(world_name, model_name, idx, model_prefix),
            camera_points(world_name, model_name, idx, model_prefix),
        ]
    elif payload in rfranger_models():
        bridges = [
            rfranger(world_name, model_name, idx, model_prefix),
        ]
    return bridges
