from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os


ld = LaunchDescription()

def get_num(context: LaunchContext, robot_num, use_rviz):
    num_str = context.perform_substitution(robot_num)
    rviz = context.perform_substitution(use_rviz)
    print(num_str)
    spawned = None
    for i in range(int(num_str)):
        x_pose = LaunchConfiguration('x_pose', default=f'{3*i}')
        y_pose = LaunchConfiguration('y_pose', default='0.0')
        spawn = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(os.path.join(get_package_share_directory('multi_robot'), 'launch'), 'spawn_turtlebot3.launch.py')
                ),
                launch_arguments={
                    'x_pose': x_pose,
                    'y_pose': y_pose,
                    'robot_prefix': f'tb{i}',
                }.items()
            )
        
        if spawned is None:
            ld.add_action(spawn)
        else:
            spawn_turtlebot3 = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawned,
                    on_exit=[spawn],
                )
            )

            ld.add_action(spawn_turtlebot3)
        spawned = spawn_turtlebot3

        spawn = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(os.path.join(get_package_share_directory('multi_robot'), 'launch'), 'navigation2.launch.py')
                ),
                launch_arguments={
                    'x_pose': x_pose,
                    'y_pose': y_pose,
                    'namespace': f'tb{i}',
                    'use_rviz': rviz,
                }.items()
            )
        
        spawn_nav2 = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawned,
                    on_exit=[spawn],
                )
            )
        ld.add_action(spawn_nav2)
        spawned = spawn_nav2


"""@brief Generate the launch descriptions for ROS
"""
def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    robot_num_arg = DeclareLaunchArgument('robot_num', default_value='2')
    robot_num = LaunchConfiguration("robot_num", default='2')

    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='False')
    use_rviz = LaunchConfiguration('use_rviz', default='False')

    function = OpaqueFunction(function=get_num, args=[LaunchConfiguration('robot_num'), LaunchConfiguration('use_rviz')])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
          launch_arguments = {
              'world': PathJoinSubstitution([get_package_share_directory('turtlebot3_gazebo'),'worlds', 'turtlebot3_world.world']),
              'gui': 'true',
          }.items()
    )

    map_server=Node(package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml'),
                     },],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ])
    
    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': 'True'},
                        {'autostart': True},
                        {'node_names': ['map_server']}])
    
    ld.add_action(robot_num_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(function)
    ld.add_action(gazebo)
    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)

    return ld