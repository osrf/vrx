from vrx_ign.bridge import Bridge, BridgeDirection


def prefix(world_name, model_name, link_name):
    return f'/world/{world_name}/model/{model_name}/link/{link_name}/sensor'


def imu(world_name, model_name, link_name='base_link'):
    sensor_prefix = prefix(world_name, model_name, link_name)
    return Bridge(
        ign_topic=f'{sensor_prefix}/imu_sensor/imu',
        ros_topic='imu/data',
        ign_type='ignition.msgs.IMU',
        ros_type='sensor_msgs/msg/Imu',
        direction=BridgeDirection.IGN_TO_ROS)


def magnetometer(world_name, model_name, link_name='base_link'):
    sensor_prefix = prefix(world_name, model_name, link_name)
    return Bridge(
        ign_topic=f'{sensor_prefix}/magnetometer/magnetometer',
        ros_topic='magnetic_field',
        ign_type='ignition.msgs.Magnetometer',
        ros_type='sensor_msgs/msg/MagneticField',
        direction=BridgeDirection.IGN_TO_ROS)


def air_pressure(world_name, model_name, link_name='base_link'):
    sensor_prefix = prefix(world_name, model_name, link_name)
    return Bridge(
        ign_topic=f'{sensor_prefix}/air_pressure/air_pressure',
        ros_topic='air_pressure',
        ign_type='ignition.msgs.FluidPressure',
        ros_type='sensor_msgs/msg/FluidPressure',
        direction=BridgeDirection.IGN_TO_ROS)


def pose(model_name):
    return Bridge(
        ign_topic=f'/model/{model_name}/pose',
        ros_topic='pose',
        ign_type='ignition.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.IGN_TO_ROS)


def pose_static(model_name):
    return Bridge(
        ign_topic=f'/model/{model_name}/pose_static',
        ros_topic='pose_static',
        ign_type='ignition.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.IGN_TO_ROS)


def cmd_vel(model_name):
    return Bridge(
        ign_topic=f'/model/{model_name}/cmd_vel',
        ros_topic='cmd_vel',
        ign_type='ignition.msgs.Twist',
        ros_type='geometry_msgs/msg/Twist',
        direction=BridgeDirection.ROS_TO_IGN)


def fixed_wing_flap(model_name, side):
    return Bridge(
        ign_topic=f'/model/{model_name}/joint/{side}_flap_joint/cmd_pos',
        ros_topic=f'cmd/{side}_flap',
        ign_type='ignition.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_IGN)


def fixed_wing_prop(model_name):
    return Bridge(
        ign_topic=f'/model/{model_name}/joint/propeller_joint/cmd_vel',
        ros_topic='cmd/motor_speed',
        ign_type='ignition.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_IGN)


def thrust(model_name, side):
    return Bridge(
        ign_topic=f'{model_name}/thrusters/{side}/thrust',
        ros_topic=f'thrusters/{side}/thrust',
        ign_type='ignition.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_IGN)


def thrust_joint_pos(model_name, side):
    # ROS naming policy indicates that first character of a name must be an alpha
    # character. In the case below, the ign topic has the joint index 0 as the
    # first char so the following topics fail to be created on the ROS end
    # left_joint_topic = '/model/' + model_name + '/joint/left_chasis_engine_joint/0/cmd_pos'
    # right_joint_topic = '/model/' + model_name + '/joint/right_chasis_engine_joint/0/cmd_pos'
    # For now, use erb to generate unique topic names in model.sdf.erb
    return Bridge(
        ign_topic=f'{model_name}/thrusters/{side}/pos',
        ros_topic=f'thrusters/{side}/pos',
        ign_type='ignition.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_IGN)


def arm_joint_states(world_name, model_name):
    arm_prefix = f'/world/{world_name}/model/{model_name}/model/arm'
    return Bridge(
        ign_topic=f'{arm_prefix}/joint_state',
        ros_topic='arm/joint_states',
        ign_type='ignition.msgs.Model',
        ros_type='sensor_msgs/msg/JointState',
        direction=BridgeDirection.IGN_TO_ROS)


def gripper_joint_states(world_name, model_name, isAttachedToArm):
    prefix = 'gripper'
    if isAttachedToArm:
        gripper_prefix = f'/world/{world_name}/model/{model_name}/model/arm/model/gripper'
        prefix = 'arm/gripper'
    else:
        gripper_prefix = f'/world/{world_name}/model/{model_name}/model/gripper'
    return Bridge(
        ign_topic=f'{gripper_prefix}/joint_state',
        ros_topic=f'{prefix}/joint_states',
        ign_type='ignition.msgs.Model',
        ros_type='sensor_msgs/msg/JointState',
        direction=BridgeDirection.IGN_TO_ROS)


def arm_joint_pos(model_name, joint_name):
    return Bridge(
        ign_topic=f'/{model_name}/arm/{joint_name}',
        ros_topic=f'arm/joint/{joint_name}/cmd_pos',
        ign_type='ignition.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_IGN)


def gripper_joint_pos(model_name, joint_name, isAttachedToArm):
    prefix = 'gripper'
    if isAttachedToArm:
        prefix = 'arm/gripper'
    return Bridge(
        ign_topic=f'/{model_name}/{prefix}/{joint_name}',
        ros_topic=f'{prefix}/joint/{joint_name}/cmd_pos',
        ign_type='ignition.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_IGN)


def wrist_joint_force_torque(model_name):
    return Bridge(
        ign_topic=f'/{model_name}/arm/wrist/forcetorque',
        ros_topic='arm/wrist/joint/wrench',
        ign_type='ignition.msgs.Wrench',
        ros_type='geometry_msgs/msg/Wrench',
        direction=BridgeDirection.IGN_TO_ROS)


def gripper_joint_force_torque(model_name, joint_name, isAttachedToArm):
    prefix = 'gripper'
    if isAttachedToArm:
        prefix = 'arm/gripper'
    return Bridge(
        ign_topic=f'/{model_name}/{prefix}/{joint_name}/forcetorque',
        ros_topic=f'{prefix}/joint/{joint_name}/wrench',
        ign_type='ignition.msgs.Wrench',
        ros_type='geometry_msgs/msg/Wrench',
        direction=BridgeDirection.IGN_TO_ROS)


def gripper_suction_contacts(model_name, isAttachedToArm):
    prefix = 'gripper'
    if isAttachedToArm:
        prefix = 'arm/gripper'
    return Bridge(
        ign_topic=f'/{model_name}/{prefix}/contact',
        ros_topic='{prefix}/contact',
        ign_type='ignition.msgs.Contacts',
        ros_type='ros_ign_interfaces/msg/Contacts',
        direction=BridgeDirection.IGN_TO_ROS
    )


def gripper_contact(model_name, isAttachedToArm, direction):
    prefix = 'gripper'
    if isAttachedToArm:
        prefix = 'arm/gripper'
    return Bridge(
        ign_topic=f'/{model_name}/{prefix}/contacts/{direction}',
        ros_topic=f'{prefix}/contacts/{direction}',
        ign_type='ignition.msgs.Boolean',
        ros_type='std_msgs/msg/Bool',
        direction=BridgeDirection.IGN_TO_ROS
    )


def gripper_suction_control(model_name, isAttachedToArm):
    prefix = 'gripper'
    if isAttachedToArm:
        prefix = 'arm/gripper'
    return Bridge(
        ign_topic=f'/{model_name}/{prefix}/suction_on',
        ros_topic=f'{prefix}/suction_on',
        ign_type='ignition.msgs.Boolean',
        ros_type='std_msgs/msg/Bool',
        direction=BridgeDirection.ROS_TO_IGN
    )


def comms_tx(model_name):
    return Bridge(
        ign_topic='/broker/msgs',
        ros_topic='tx',
        ign_type='ignition.msgs.Dataframe',
        ros_type='ros_ign_interfaces/msg/Dataframe',
        direction=BridgeDirection.ROS_TO_IGN)


def comms_rx(model_name):
    return Bridge(
        ign_topic=f'/model/{model_name}/rx',
        ros_topic='rx',
        ign_type='ignition.msgs.Dataframe',
        ros_type='ros_ign_interfaces/msg/Dataframe',
        direction=BridgeDirection.IGN_TO_ROS)


def score():
    return Bridge(
        ign_topic='/vrx/score',
        ros_topic='/vrx/score',
        ign_type='ignition.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.IGN_TO_ROS)


def clock():
    return Bridge(
        ign_topic='/clock',
        ros_topic='/clock',
        ign_type='ignition.msgs.Clock',
        ros_type='rosgraph_msgs/msg/Clock',
        direction=BridgeDirection.IGN_TO_ROS)


def run_clock():
    return Bridge(
        ign_topic='/vrx/run_clock',
        ros_topic='/vrx/run_clock',
        ign_type='ignition.msgs.Clock',
        ros_type='rosgraph_msgs/msg/Clock',
        direction=BridgeDirection.IGN_TO_ROS)


def phase():
    return Bridge(
        ign_topic='/vrx/phase',
        ros_topic='/vrx/phase',
        ign_type='ignition.msgs.StringMsg',
        ros_type='std_msgs/msg/String',
        direction=BridgeDirection.IGN_TO_ROS)


def stream_status():
    return Bridge(
        ign_topic='/vrx/target/stream/status',
        ros_topic='/vrx/target/stream/status',
        ign_type='ignition.msgs.StringMsg',
        ros_type='std_msgs/msg/String',
        direction=BridgeDirection.IGN_TO_ROS)
