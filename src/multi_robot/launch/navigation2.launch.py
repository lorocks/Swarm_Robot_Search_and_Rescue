# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


ld = LaunchDescription()


def launch(context: LaunchContext, x_pose, y_pose, namespace, use_rviz):
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    package_dir = get_package_share_directory('multi_robot')
    nav_dir = get_package_share_directory('nav2_bringup')

    x_pos = context.perform_substitution(x_pose)
    y_pos = context.perform_substitution(y_pose)
    namespace_r = context.perform_substitution(namespace)

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            package_dir,
            'map',
            'map.yaml'))

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(package_dir, 'param', 'waffle.yaml'))
    
    nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_dir, 'launch', 'bringup_launch.py')),
                launch_arguments={  
                    'slam': 'False',
                    'namespace': namespace,
                    'use_namespace': 'True',
                    'map': '',
                    'map_server': 'False',
                    'params_file': param_dir,
                    'default_bt_xml_filename': os.path.join(
                        get_package_share_directory('nav2_bt_navigator'),
                        'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                    'autostart': 'true',
                    'use_sim_time': use_sim_time, 'log_level': 'warn'
                }.items()
            )

    message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
            x_pos + ', y: ' + y_pos + \
            ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'
    
    name = [ '/' +  namespace_r]

    initial_pose = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', name + ['/initialpose'],
                'geometry_msgs/PoseWithCovarianceStamped', message],
            output='screen'
        )
    
    rviz_nav = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_dir, 'launch', 'rviz_launch.py')),
                    launch_arguments={
                        'use_sim_time': use_sim_time, 
                        'namespace': namespace,
                        'use_namespace': 'True',
                    }.items(),
                    condition=IfCondition(use_rviz)
                )

    print('hoalla')
    ld.add_action(nav2)
    ld.add_action(rviz_nav)
    ld.add_action(initial_pose)


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='False')
    use_rviz = LaunchConfiguration('use_rviz', default='False')

    namespace_arg = DeclareLaunchArgument('namespace', default_value='tb0')
    namespace = LaunchConfiguration('namespace', default='tb0')

    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    x_pose = LaunchConfiguration('x_pose')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    y_pose = LaunchConfiguration('y_pose')

    function = OpaqueFunction(function=launch, args=[LaunchConfiguration('x_pose'), LaunchConfiguration('y_pose'), LaunchConfiguration('namespace'), LaunchConfiguration('use_rviz')])

    ld.add_action(use_rviz_arg)
    ld.add_action(namespace_arg)
    ld.add_action(x_pose_arg)
    ld.add_action(y_pose_arg)
    ld.add_action(function)

    return ld
