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

n = 2

def get_num(context: LaunchContext, robot_num):
    num_str = context.perform_substitution(robot_num)
    n = int(num_str)
    print("done")

"""@brief Generate the launch descriptions for ROS
"""
def generate_launch_description():
    robot_num_arg = DeclareLaunchArgument('robot_num', default_value='2')
    robot_num = LaunchConfiguration("robot_num", default='2')

    function = OpaqueFunction(function=get_num, args=[LaunchConfiguration('robot_num')])

    launch_file_dir = os.path.join(get_package_share_directory('multi_robot'), 'launch')

    spawn_robots = []
    for i in range(n):
        x_pose = LaunchConfiguration('x_pose', default=f'{3*i}')
        y_pose = LaunchConfiguration('y_pose', default='0.0')
        spawn_robots.append(
                IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
                ),
                launch_arguments={
                    'x_pose': x_pose,
                    'y_pose': y_pose,
                    'robot_prefix': f'tb{i}',
                }.items()
            )
        )
    
    # spawn_robots = [ IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
    #             ),
    #             launch_arguments={
    #                 'x_pose': f'{i}',
    #                 'y_pose': '0.0',
    #                 'robot_prefix': f'tb{i}',
    #             }.items()
    #         ) for i in range(n)]

    # delay_spawnning = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=function,
    #         on_exit=[spawn_robots],
    #     )
    # )

    ld = LaunchDescription()
    
    ld.add_action(robot_num_arg)
    ld.add_action(function)
    for robot in spawn_robots:
        ld.add_action(robot)

    return ld