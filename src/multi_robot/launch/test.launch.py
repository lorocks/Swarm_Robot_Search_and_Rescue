import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, FindExecutable
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import launch.logging

def generate_launch_description():
    ld = LaunchDescription()

    robots = [
        {'name': 'tb1', 'x_pose': '-1.5', 'y_pose': '-0.5', 'z_pose': 0.01},
        {'name': 'tb2', 'x_pose': '-1.5', 'y_pose': '0.5', 'z_pose': 0.01},
        # {'name': 'tb3', 'x_pose': '-1.5', 'y_pose': '0.0', 'z_pose': 0.01},
        # {'name': 'tb4', 'x_pose': '-1.0', 'y_pose': '0.5', 'z_pose': 0.01},
        # ...
        # ...
        ]

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    use_rviz = LaunchConfiguration('use_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=use_rviz, description='Enable rviz launch'
    )

    
    package_dir = get_package_share_directory('multi_robot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')


    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            package_dir, 'rviz', 'multi_nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    urdf = os.path.join(
        package_dir, 'urdf', 'turtlebot3_waffle.urdf'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
          launch_arguments = {
              'world': PathJoinSubstitution([get_package_share_directory('turtlebot3_gazebo'),'worlds', 'turtlebot3_world.world']),
              'gui': 'true',
          }.items()
    )

    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(package_dir, 'param', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
     
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(gazebo)
 
    map_server=Node(package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(package_dir, 'map', 'map.yaml'),
                     },],
        remappings=[('/tf', 'tf'),
                  ('/tf_static', 'tf_static')])

    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])


    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)

    last_action = None
    for robot in robots:

        namespace = [ '/' + robot['name'] ]

        robot_state_publisher = Node(
            package='robot_state_publisher',
            namespace=namespace,
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                            'publish_frequency': 10.0}],
            remappings=[('/tf', 'tf'),
                  ('/tf_static', 'tf_static')],
            arguments=[urdf],
        )

        spawn_turtlebot3_burger = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', os.path.join(package_dir,'model', 'turtlebot3_waffle', 'model.sdf'),
                '-entity', robot['name'],
                '-robot_namespace', namespace,
                '-x', robot['x_pose'], '-y', robot['y_pose'],
                '-z', '0.01', '-Y', '0.0',
                '-unpause',
            ],
            output='screen',
        )

        bringup_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_dir, 'launch', 'bringup_launch.py')),
                    launch_arguments={  
                                    'slam': 'False',
                                    'namespace': namespace,
                                    'use_namespace': 'True',
                                    'map': '',
                                    'map_server': 'False',
                                    'params_file': params_file,
                                    'default_bt_xml_filename': os.path.join(
                                        get_package_share_directory('nav2_bt_navigator'),
                                        'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                                    'autostart': 'true',
                                    'use_sim_time': use_sim_time, 'log_level': 'warn'}.items()
                                    )

        if last_action is None:
            ld.add_action(robot_state_publisher)
            ld.add_action(spawn_turtlebot3_burger)
            ld.add_action(bringup_cmd)

        else:
            spawn_turtlebot3_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[spawn_turtlebot3_burger,
                            robot_state_publisher,
                            bringup_cmd],
                )
            )

            ld.add_action(spawn_turtlebot3_event)

        last_action = spawn_turtlebot3_burger

    for robot in robots:

        namespace = [ '/' + robot['name'] ]

        # Create a initial pose topic publish call
        message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
            robot['x_pose'] + ', y: ' + robot['y_pose'] + \
            ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'

        initial_pose_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', namespace + ['/initialpose'],
                'geometry_msgs/PoseWithCovarianceStamped', message],
            output='screen'
        )

        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_dir, 'launch', 'rviz_launch.py')),
                launch_arguments={'use_sim_time': use_sim_time, 
                                  'namespace': namespace,
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file, 'log_level': 'warn'}.items(),
                                   condition=IfCondition(use_rviz)
                                    )


        post_spawn_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=last_action,
                on_exit=[initial_pose_cmd, rviz_cmd],
            )
        )

        last_action = initial_pose_cmd

        ld.add_action(post_spawn_event)

    for robot in robots:
        run_goal = ExecuteProcess(
            cmd=['ros2', 'run', 'multi_robot', 'goal_pub', robot['name']],
            shell = True
        )


        post_node_run_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=last_action,
                on_exit=[run_goal],
            )
        )

        ld.add_action(post_node_run_event)      
    

    return ld