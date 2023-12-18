from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

"""@brief Generate the launch descriptions for ROS
"""
def generate_launch_description():
    desc = LaunchDescription()

    robot_num = DeclareLaunchArgument(name="robot_num", default_value='2')

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    n = 5
    x_pose = LaunchConfiguration('x_pose', default=f'{n}')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'turtlebot3_house.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )


    desc.add_action(spawn_turtlebot)
    desc.add_action(robot_num)

    return desc