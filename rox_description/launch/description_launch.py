# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, PythonExpression
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from launch.conditions import IfCondition
import os
from pathlib import Path
import xacro

def execution_stage(context: LaunchContext, rox_type, arm_type):
    arm_typ = arm_type.perform(context)
    rox_typ = rox_type.perform(context)
    
    urdf = os.path.join(get_package_share_directory('rox_description'), 'urdf', 'rox.urdf.xacro')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command([
            "xacro", " ", urdf, " ", 'arm_type:=',
            arm_typ,
            " ", 'rox_type:=',
            rox_type])}],
        arguments=[urdf])

    return [start_robot_state_publisher_cmd]

def generate_launch_description():
    
    # Launch configuration
    rox_type = LaunchConfiguration('rox_type')
    arm_type = LaunchConfiguration('arm_type')
    
    declare_rox_type_cmd = DeclareLaunchArgument(
            'rox_type', default_value='argo',
            description='Robot type - Options: argo/argo-trio/diff/trike'
        )
   
    declare_arm_cmd = DeclareLaunchArgument(
            'arm_type', default_value='',
            description='Arm used in the robot - currently only support universal'
        )

    context_arguments = [rox_type, arm_type]

    opq_function = OpaqueFunction(function=execution_stage,
                                  args=context_arguments)

    ld = LaunchDescription()
    ld.add_action(declare_rox_type_cmd)
    ld.add_action(declare_arm_cmd)
    ld.add_action(opq_function)

    return ld
