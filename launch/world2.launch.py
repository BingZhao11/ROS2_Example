# 封装终端指令相关类-------------
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
# 参数声明与获取----------------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# 文件包含相关------------------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 获取功能包下share目录的路径-----
from ament_index_python.packages import get_package_share_directory
# 分组相关----------------------
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
# 事件相关----------------------
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.actions import RegisterEventHandler, LogInfo

from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    robot_name = 'camera_calibration'
    checker_name = 'checker_name'
    world_file_name = 'Gravity_free.world'

    world = os.path.join(get_package_share_directory(robot_name), 'world', world_file_name)
    urdf = os.path.join(get_package_share_directory(robot_name), 'urdf', 'd435i.urdf')
    checkerboad = os.path.join(get_package_share_directory(robot_name),'urdf', 'small_checkerboard.sdf')
    
    # launch gazebo
    start_gazebo_cmd = ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output = 'screen')

    # load the robot 
    spawn_entity_cmd_1 = Node(
        name='spawn_entity_cmd_1',
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-file', urdf
        ],
        output='screen'
    )

    spawn_entity_cmd_2 = Node(
        name='spawn_entity_cmd_2',
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', checker_name,
            '-file', checkerboad,
            '-x', '-0.03',
            '-y', '-0.03',
            '-z', '0.2'
        ],
        output='screen'
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf]
    )

    start_robot_joint_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        arguments=[urdf]
    )


    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd_1)
    ld.add_action(spawn_entity_cmd_2)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_robot_joint_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld

