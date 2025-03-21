# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from launch.conditions import IfCondition, UnlessCondition
import os
from pathlib import Path
import xacro

def execution_stage(context: LaunchContext,
                    robot_namespace,
                    rox_type,
                    arm_type,
                    scanner_type,
                    use_imu,
                    ur_dc,
                    mock_arm,
                    ):
    
    rox = get_package_share_directory('rox_bringup')

    arm_typ = str(arm_type.perform(context))
    rox_typ = str(rox_type.perform(context))
    scanner_typ = str(scanner_type.perform(context))
    imu_enable = str(use_imu.perform(context))
    use_ur_dc = ur_dc.perform(context)
    use_mock = str(mock_arm.perform(context))
    joint_type = "revolute"

    if use_mock:
        joint_type = "fixed"

    launches = []
    
    rp_ns = ""
    if (robot_namespace.perform(context) != "/"):
        rp_ns = robot_namespace.perform(context) + "/"
    
    urdf = os.path.join(get_package_share_directory('rox_description'), 'urdf', 'rox.urdf.xacro')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                "xacro", " ", urdf,
                " ", 'arm_type:=', arm_typ,
                " ", 'rox_type:=', rox_typ,
                " ", 'use_mock_hardware:=', use_mock,  # experimental
                " ", 'scanner:=', scanner_typ,
                " ", 'use_imu:=', imu_enable,
                " ", 'use_ur_dc:=', use_ur_dc,
                " ", 'joint_type:=', joint_type,
            ]),
            'frame_prefix': rp_ns
        }],
        arguments=[urdf]
    )
    
    launches.append(start_robot_state_publisher_cmd)

    # 1. Relayboard
    relayboard = Node(
        package='neo_relayboard_v3', 
        executable='relayboardv3_node',
        output='screen',
        name='neo_relayboard_v3_node',
        parameters = [
            {"pilot_config": "/home/neobotix/ros2_workspace/src/rox/rox_bringup/configs/neo_relayboard_v3/rox-" + rox_typ + "/"}
        ],
        condition=UnlessCondition(mock_arm)
    )
    
    launches.append(relayboard)
                    
    # 2. Kinematics
    if (rox_typ == "argo"):
        kinematics = Node(
            package='rox_argo_kinematics',
            executable='rox_argo_kinematics_node',
            output='screen',
            name='argo_kinematics_node',
            parameters = [os.path.join(rox,'configs/kinematics', f'{rox_typ}_kinematics.yaml')],
            condition=UnlessCondition(mock_arm)
        )

        launches.append(kinematics)
    
    if (rox_typ == "diff"):
        kinematics = Node(
            package='rox_diff_kinematics',
            executable='rox_diff_kinematics_node',
            output='screen',
            name='diff_kinematics_node',
            parameters = [os.path.join(rox,'configs/kinematics', f'{rox_typ}_kinematics.yaml')],
            condition=UnlessCondition(mock_arm)
        )

        launches.append(kinematics)

    # 3. Teleop
    teleop = Node(
        package='neo_teleop2',
        executable='neo_teleop2_node',
        output='screen',
        name='neo_teleop2_node',
        parameters = [os.path.join(rox,'configs/teleop', f'{rox_typ}_teleop.yaml')],
        condition=UnlessCondition(mock_arm)
    )

    launches.append(teleop)

    # Joy
    joy = Node(
            package='joy', 
            executable='joy_node', 
            output='screen',
            name='joy_node',
            parameters = [{'dev': "/dev/input/js0"}, {'deadzone':0.12}],
            condition=UnlessCondition(mock_arm)
        )

    launches.append(joy)

    # 4. Laser - Nanoscan
    if scanner_typ == "nanoscan":
        scan1 = Node(
                package="sick_safetyscanners2",
                executable="sick_safetyscanners2_node",
                name="lidar_1_node",
                output="screen",
                emulate_tty=True,
                parameters=[os.path.join(rox, 'configs/sick_lidar', 'nanoscan_1.yaml')],
                condition=UnlessCondition(mock_arm),
                remappings=[
                    ('/scan', '/lidar_1/scan_filtered')
                ]
            )
        
        launches.append(scan1)

        scan2 = Node(
                package="sick_safetyscanners2",
                executable="sick_safetyscanners2_node",
                name="lidar_2_node",
                output="screen",
                emulate_tty=True,
                parameters=[os.path.join(rox, 'configs/sick_lidar', 'nanoscan_2.yaml')],
                condition=UnlessCondition(mock_arm),
                remappings=[
                    ('/scan', '/lidar_2/scan_filtered'),
                ]
            )
        
        launches.append(scan2)
    
    # Laser - PsenScan
    elif scanner_typ == "psenscan":
        scan = IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(get_package_share_directory('psen_scan_v2'),
                    'launch',
                    'psen_scan_v2.launch.xml')
            ),
            condition=UnlessCondition(mock_arm),
            launch_arguments={
                'sensor_ip': "192.168.1.30",
                'host_ip': "192.168.1.10"
            }.items()
        )
    
        launches.append(scan)

    # 5. IMU
    if imu_enable == 'True':
        imu = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rox, 'configs/phidget_imu', 'imu_launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items(),
            condition=UnlessCondition(mock_arm)
        )
        launches.append(imu)

    # 6. Arm - Bringing up drivers for Universal Arm
    # TODO: Add support for Elite Robots
    # TODO: Add support for namespacing
    if (arm_typ == "ur5" or
        arm_typ == "ur10" or
        arm_typ == "ur5e" or
        arm_typ == "ur10e"):
        
        ur_arm = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rox,
                        'configs/ur',
                        'ur_control.launch.py') 
                ),
                launch_arguments={
                    'ur_type': arm_typ,
                    'robot_ip': "192.168.1.102",
                    'tf_prefix': arm_typ,
                    'use_mock_hardware': use_mock,
                    'mock_sensor_commands': use_mock
                }.items()
            )

        launches.append(ur_arm)

    # Relaying lidar data to /scan topic
    relay_topic_lidar1 = Node(
        package='topic_tools',
        executable='relay',
        name='relay_lidar1',
        namespace=rp_ns,
        output='screen',
        parameters=[{'input_topic': rp_ns + "lidar_1/scan_filtered",'output_topic': rp_ns + "scan"}],
        condition=UnlessCondition(mock_arm)
    )

    relay_topic_lidar2 = Node(
        package='topic_tools',
        executable='relay',
        name='relay_lidar2',
        namespace=rp_ns,
        output='screen',
        parameters=[{'input_topic': rp_ns + "lidar_2/scan_filtered",'output_topic': rp_ns + "scan"}],
        condition=UnlessCondition(mock_arm)
    )

    # Relay drive/joint_states topic to joint_states
    relay_topic_joint_states = Node(
        package='topic_tools',
        executable='relay',
        name='relay_joint_states',
        output='screen',
        parameters=[{'input_topic': "drive/joint_states",'output_topic': "joint_states"}],
        condition=UnlessCondition(mock_arm)
    )

    launches.append(relay_topic_lidar1)
    launches.append(relay_topic_lidar2)
    launches.append(relay_topic_joint_states)

    return launches

def generate_launch_description():
    
    # Launch configuration
    robot_namespace = LaunchConfiguration('robot_namespace')
    rox_type = LaunchConfiguration('rox_type')
    arm_type = LaunchConfiguration('arm_type')
    scanner_type = LaunchConfiguration('scanner_type')
    imu_enable = LaunchConfiguration('imu_enable')
    ur_dc = LaunchConfiguration('use_ur_dc')
    mock_arm = LaunchConfiguration('use_mock_arm')

    context_arguments = [robot_namespace, rox_type, arm_type, scanner_type, imu_enable, ur_dc, mock_arm]

    opq_function = OpaqueFunction(function=execution_stage, args=context_arguments)
    
    declare_namespace_cmd = DeclareLaunchArgument(
            'robot_namespace', default_value='', description='Top-level namespace'
        )
    
    declare_rox_type_cmd = DeclareLaunchArgument(
            'rox_type', default_value='argo',
            description='Robot type - Options: argo/diff/trike'
        )

    declare_imu_cmd = DeclareLaunchArgument(
            'imu_enable', default_value='False',
            description='Enable IMU - Options: True/False'
        )
   
    declare_arm_cmd = DeclareLaunchArgument(
            'arm_type', default_value='',
            description='Arm used in the robot - currently only support universal'
        )
    
    declare_scanner_cmd = DeclareLaunchArgument(
            'scanner_type', default_value='nanoscan',
            description='Scanner options available: nanoscan/psenscan'
        )
    
    declare_ur_pwr_variant_cmd = DeclareLaunchArgument(
            'use_ur_dc', default_value='false',
            description='Set this argument to True if you have an UR arm with DC variant'
        )
    
    declare_mock_arm_cmd = DeclareLaunchArgument(
            'use_mock_arm', default_value='False',
            description="Mock arm and gripper (if available)"
        )
    
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_rox_type_cmd)
    ld.add_action(declare_imu_cmd)
    ld.add_action(declare_arm_cmd)
    ld.add_action(declare_scanner_cmd)
    ld.add_action(declare_ur_pwr_variant_cmd)
    ld.add_action(declare_mock_arm_cmd)
    ld.add_action(opq_function)

    return ld