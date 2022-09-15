from vrx_gz.bridge import Bridge, BridgeDirection


def prefix(world_name, model_name, link_name):
    return f'/world/{world_name}/model/{model_name}/link/{link_name}/sensor'


def imu(world_name, model_name, link_name='base_link'):
    sensor_prefix = prefix(world_name, model_name, link_name)
    return Bridge(
        gz_topic=f'{sensor_prefix}/imu_sensor/imu',
        ros_topic='imu/data',
        gz_type='gz.msgs.IMU',
        ros_type='sensor_msgs/msg/Imu',
        direction=BridgeDirection.GZ_TO_ROS)


def magnetometer(world_name, model_name, link_name='base_link'):
    sensor_prefix = prefix(world_name, model_name, link_name)
    return Bridge(
        gz_topic=f'{sensor_prefix}/magnetometer/magnetometer',
        ros_topic='magnetic_field',
        gz_type='gz.msgs.Magnetometer',
        ros_type='sensor_msgs/msg/MagneticField',
        direction=BridgeDirection.GZ_TO_ROS)


def air_pressure(world_name, model_name, link_name='base_link'):
    sensor_prefix = prefix(world_name, model_name, link_name)
    return Bridge(
        gz_topic=f'{sensor_prefix}/air_pressure/air_pressure',
        ros_topic='air_pressure',
        gz_type='gz.msgs.FluidPressure',
        ros_type='sensor_msgs/msg/FluidPressure',
        direction=BridgeDirection.GZ_TO_ROS)


def pose(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/pose',
        ros_topic='pose',
        gz_type='gz.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.GZ_TO_ROS)


def pose_static(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/pose_static',
        ros_topic='pose_static',
        gz_type='gz.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.GZ_TO_ROS)


def cmd_vel(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/cmd_vel',
        ros_topic='cmd_vel',
        gz_type='gz.msgs.Twist',
        ros_type='geometry_msgs/msg/Twist',
        direction=BridgeDirection.ROS_TO_GZ)


def fixed_wing_flap(model_name, side):
    return Bridge(
        gz_topic=f'/model/{model_name}/joint/{side}_flap_joint/cmd_pos',
        ros_topic=f'cmd/{side}_flap',
        gz_type='gz.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_GZ)


def fixed_wing_prop(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/joint/propeller_joint/cmd_vel',
        ros_topic='cmd/motor_speed',
        gz_type='gz.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_GZ)


def thrust(model_name, side):
    return Bridge(
        gz_topic=f'{model_name}/thrusters/{side}/thrust',
        ros_topic=f'thrusters/{side}/thrust',
        gz_type='gz.msgs.Double',
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
        gz_type='gz.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_GZ)


def arm_joint_states(world_name, model_name):
    arm_prefix = f'/world/{world_name}/model/{model_name}/model/arm'
    return Bridge(
        gz_topic=f'{arm_prefix}/joint_state',
        ros_topic='arm/joint_states',
        gz_type='gz.msgs.Model',
        ros_type='sensor_msgs/msg/JointState',
        direction=BridgeDirection.GZ_TO_ROS)


def gripper_joint_states(world_name, model_name, isAttachedToArm):
    prefix = 'gripper'
    if isAttachedToArm:
        gripper_prefix = f'/world/{world_name}/model/{model_name}/model/arm/model/gripper'
        prefix = 'arm/gripper'
    else:
        gripper_prefix = f'/world/{world_name}/model/{model_name}/model/gripper'
    return Bridge(
        gz_topic=f'{gripper_prefix}/joint_state',
        ros_topic=f'{prefix}/joint_states',
        gz_type='gz.msgs.Model',
        ros_type='sensor_msgs/msg/JointState',
        direction=BridgeDirection.GZ_TO_ROS)


def arm_joint_pos(model_name, joint_name):
    return Bridge(
        gz_topic=f'/{model_name}/arm/{joint_name}',
        ros_topic=f'arm/joint/{joint_name}/cmd_pos',
        gz_type='gz.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_GZ)


def gripper_joint_pos(model_name, joint_name, isAttachedToArm):
    prefix = 'gripper'
    if isAttachedToArm:
        prefix = 'arm/gripper'
    return Bridge(
        gz_topic=f'/{model_name}/{prefix}/{joint_name}',
        ros_topic=f'{prefix}/joint/{joint_name}/cmd_pos',
        gz_type='gz.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_GZ)


def wrist_joint_force_torque(model_name):
    return Bridge(
        gz_topic=f'/{model_name}/arm/wrist/forcetorque',
        ros_topic='arm/wrist/joint/wrench',
        gz_type='gz.msgs.Wrench',
        ros_type='geometry_msgs/msg/Wrench',
        direction=BridgeDirection.GZ_TO_ROS)


def gripper_joint_force_torque(model_name, joint_name, isAttachedToArm):
    prefix = 'gripper'
    if isAttachedToArm:
        prefix = 'arm/gripper'
    return Bridge(
        gz_topic=f'/{model_name}/{prefix}/{joint_name}/forcetorque',
        ros_topic=f'{prefix}/joint/{joint_name}/wrench',
        gz_type='gz.msgs.Wrench',
        ros_type='geometry_msgs/msg/Wrench',
        direction=BridgeDirection.GZ_TO_ROS)


def gripper_suction_contacts(model_name, isAttachedToArm):
    prefix = 'gripper'
    if isAttachedToArm:
        prefix = 'arm/gripper'
    return Bridge(
        gz_topic=f'/{model_name}/{prefix}/contact',
        ros_topic='{prefix}/contact',
        gz_type='gz.msgs.Contacts',
        ros_type='ros_gz_interfaces/msg/Contacts',
        direction=BridgeDirection.GZ_TO_ROS
    )


def gripper_contact(model_name, isAttachedToArm, direction):
    prefix = 'gripper'
    if isAttachedToArm:
        prefix = 'arm/gripper'
    return Bridge(
        gz_topic=f'/{model_name}/{prefix}/contacts/{direction}',
        ros_topic=f'{prefix}/contacts/{direction}',
        gz_type='gz.msgs.Boolean',
        ros_type='std_msgs/msg/Bool',
        direction=BridgeDirection.GZ_TO_ROS
    )


def gripper_suction_control(model_name, isAttachedToArm):
    prefix = 'gripper'
    if isAttachedToArm:
        prefix = 'arm/gripper'
    return Bridge(
        gz_topic=f'/{model_name}/{prefix}/suction_on',
        ros_topic=f'{prefix}/suction_on',
        gz_type='gz.msgs.Boolean',
        ros_type='std_msgs/msg/Bool',
        direction=BridgeDirection.ROS_TO_GZ
    )


def comms_tx(model_name):
    return Bridge(
        gz_topic='/broker/msgs',
        ros_topic='tx',
        gz_type='gz.msgs.Dataframe',
        ros_type='ros_gz_interfaces/msg/Dataframe',
        direction=BridgeDirection.ROS_TO_GZ)


def comms_rx(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/rx',
        ros_topic='rx',
        gz_type='gz.msgs.Dataframe',
        ros_type='ros_gz_interfaces/msg/Dataframe',
        direction=BridgeDirection.GZ_TO_ROS)


def score():
    return Bridge(
        gz_topic='/vrx/score',
        ros_topic='/vrx/score',
        gz_type='gz.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)


def clock():
    return Bridge(
        gz_topic='/clock',
        ros_topic='/clock',
        gz_type='gz.msgs.Clock',
        ros_type='rosgraph_msgs/msg/Clock',
        direction=BridgeDirection.GZ_TO_ROS)


def run_clock():
    return Bridge(
        gz_topic='/vrx/run_clock',
        ros_topic='/vrx/run_clock',
        gz_type='gz.msgs.Clock',
        ros_type='rosgraph_msgs/msg/Clock',
        direction=BridgeDirection.GZ_TO_ROS)


def phase():
    return Bridge(
        gz_topic='/vrx/phase',
        ros_topic='/vrx/phase',
        gz_type='gz.msgs.StringMsg',
        ros_type='std_msgs/msg/String',
        direction=BridgeDirection.GZ_TO_ROS)


def stream_status():
    return Bridge(
        gz_topic='/vrx/target/stream/status',
        ros_topic='/vrx/target/stream/status',
        gz_type='gz.msgs.StringMsg',
        ros_type='std_msgs/msg/String',
        direction=BridgeDirection.GZ_TO_ROS)
