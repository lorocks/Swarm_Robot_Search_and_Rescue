from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os


ld = LaunchDescription()

def get_num(context: LaunchContext, robot_num):
    num_str = context.perform_substitution(robot_num)
    spawn_robots = []
    for i in range(int(num_str)):
        x_pose = LaunchConfiguration('x_pose', default=f'{3*i}')
        y_pose = LaunchConfiguration('y_pose', default='0.0')
        spawn_robots.append(
                IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(os.path.join(get_package_share_directory('multi_robot'), 'launch'), 'spawn_turtlebot3.launch.py')
                ),
                launch_arguments={
                    'x_pose': x_pose,
                    'y_pose': y_pose,
                    'robot_prefix': f'tb{i}',
                }.items()
            )
        )
    for robot in spawn_robots:
        ld.add_action(robot)
    

"""@brief Generate the launch descriptions for ROS
"""
def generate_launch_description():
    robot_num_arg = DeclareLaunchArgument('robot_num', default_value='2')
    robot_num = LaunchConfiguration("robot_num", default='2')

    function = OpaqueFunction(function=get_num, args=[LaunchConfiguration('robot_num')])
    
    ld.add_action(robot_num_arg)
    ld.add_action(function)

    return ld