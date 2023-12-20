# Copyright 2023 Lowell Lobo, Mayank Deshpande

# Permission is hereby granted, free of charge, to any person obtaining a copy 
# of this software and associated documentation files (the “Software”), to deal 
# in the Software without restriction, including without limitation the rights to 
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies 
# of the Software, and to permit persons to whom the Software is furnished to do 
# so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all 
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
# WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='')
    use_sim_time = LaunchConfiguration('use_sim_time')

    namespace_arg = DeclareLaunchArgument('namespace', default_value='tb0')
    namespace = LaunchConfiguration('namespace', default='tb0')

    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='-2.0')
    x_pose = LaunchConfiguration('x_pose')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    y_pose = LaunchConfiguration('y_pose')

    package_dir = get_package_share_directory('multi_robot')


    # Obtain urdf from xacro files.
    package_dir = get_package_share_directory('multi_robot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    urdf_file_path = os.path.join(package_dir, 'urdf', 'turtlebot3_waffle.urdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
          launch_arguments = {
              'world': PathJoinSubstitution([get_package_share_directory('turtlebot3_gazebo'),'worlds', 'turtlebot3_world.world']),
              'gui': 'true',
          }.items()
    )

    # robot_state_publisher = Node(
    #         package='robot_state_publisher',
    #         namespace=namespace,
    #         executable='robot_state_publisher',
    #         output='screen',
    #         parameters=[{'use_sim_time': use_sim_time,
    #                         'publish_frequency': 10.0}],
    #         remappings=[
    #         ('/tf', 'tf'),
    #         ('/tf_static', 'tf_static')
    #     ],
    #         arguments=[urdf_file_path],
    #     )

    # # Spawn robot
    # spawn_turtlebot3 = Node(
    #         package='gazebo_ros',
    #         executable='spawn_entity.py',
    #         arguments=[
    #             '-file', os.path.join(package_dir, 'model', 'turtlebot3_waffle', 'model.sdf'),
    #             '-entity', namespace,
    #             '-robot_namespace', namespace,
    #             '-x', x_pose, '-y', y_pose,
    #             '-z', '0.01', '-Y', '0.0',
    #             '-unpause',
    #         ],
    #         output='screen',
    #     )

    # map_server=Node(package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{'yaml_filename': os.path.join(package_dir, 'map', 'map.yaml'),
    #                  },],
    #     remappings=[
    #         ('/tf', 'tf'),
    #         ('/tf_static', 'tf_static')
    #     ])
    
    # map_server_lifecyle=Node(package='nav2_lifecycle_manager',
    #         executable='lifecycle_manager',
    #         name='lifecycle_manager_map_server',
    #         output='screen',
    #         parameters=[{'use_sim_time': True},
    #                     {'autostart': True},
    #                     {'node_names': ['map_server']}])

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(x_pose_arg)
    ld.add_action(y_pose_arg)
    ld.add_action(use_sim_time_arg)
    # ld.add_action(robot_state_publisher)
    ld.add_action(namespace_arg)
    # ld.add_action(spawn_turtlebot3)
    ld.add_action(gazebo)
    # ld.add_action(map_server)
    # ld.add_action(map_server_lifecyle)

    return ld
