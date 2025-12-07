import os
import launch
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, EmitEvent, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch_ros.actions import Node


def launch_robots(context: LaunchContext, *args, **kwargs):
    robot_launch_file = kwargs['robot_launch_file']
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    actions = []
    for i in range(num_robots):
        robot_id = f'robot_{i+1}'
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch_file),
                launch_arguments={
                    'robot_id': robot_id,
                    'use_sim_time': use_sim_time,
                }.items()
            )
        )
    return actions

# Instantiate WebotsLauncher and its supervisor, and return those actions + shutdown handler.
def build_and_launch_webots(context: LaunchContext, *args, **kwargs):
    fleet_package = kwargs.get('fleet_package')

    world_file = LaunchConfiguration('world_file').perform(context)
    package_dir = get_package_share_directory(fleet_package)
    world_path = os.path.join(package_dir, 'worlds', world_file)

    webots = WebotsLauncher(world=world_path, ros2_supervisor=True)

    shutdown_on_webots_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=webots,
            on_exit=[EmitEvent(event=launch.events.Shutdown())],
        )
    )

    # Return the actions to be added to the launch description
    return [webots, webots._supervisor, shutdown_on_webots_exit]


def generate_launch_description():
    fleet_package = 'mars_fleet_bringup'
    fleet_package_dir = get_package_share_directory(fleet_package)
    robot_launch_file = os.path.join(
        get_package_share_directory(fleet_package),
        'launch',
        'robot_instance.launch.py',
    )

    # Launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file', default_value='mars_2_robots.wbt', description='Name of the world file inside <pkg>/worlds/'
    )
    num_robots_arg = DeclareLaunchArgument(
        'num_robots', default_value='2', description='Number of robots to launch'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Webots) clock'
    )
    record_rosbag_arg = DeclareLaunchArgument(
        'record_rosbag', default_value='false', description='Enable rosbag recording (true/false)'
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='true', description='Whether to launch RViz2'
    )

    record_rosbag = LaunchConfiguration('record_rosbag')
    launch_rviz = LaunchConfiguration('launch_rviz')

    # Build LaunchDescription and add args
    ld = LaunchDescription([
        world_file_arg,
        num_robots_arg,
        use_sim_time_arg,
        record_rosbag_arg,
        launch_rviz_arg,
    ])

    ld.add_action(OpaqueFunction(function=build_and_launch_webots, kwargs={'fleet_package': fleet_package}))

    ld.add_action(OpaqueFunction(function=launch_robots, kwargs={'robot_launch_file': robot_launch_file}))

    # Optional global rosbag recording
    rosbag_recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-x', '/camera/.*'],
        output='screen',
        condition=IfCondition(record_rosbag),
    )
    ld.add_action(rosbag_recorder)
    
    
    # -------------------------------------------------------
    # Overseer Node (mars_overseer)
    # -------------------------------------------------------
    #overseer_pkg_dir = get_package_share_directory('mars_overseer')

    overseer_node = Node(
        package='mars_overseer',
        executable='overseer_node',
        name='overseer',
        output='log',
        parameters=[{'use_sim_time': True}],
    )
    ld.add_action(overseer_node)

    # -------------------------------------------------------
    # RViz2 Launch
    # -------------------------------------------------------
    rviz_config = os.path.join(fleet_package_dir, 'config', 'mars_config.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(launch_rviz),
    )
    ld.add_action(rviz_node)

    # -------------------------------------------------------
    # Static TF: global_map -> robot_1/map
    # anchor robot_1/map at the global origin
    # -------------------------------------------------------
    static_tf_robot1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='global_to_robot1_map',
        output='screen',
        arguments=[
            '0.0', '0.0', '0.0',      # x y z
            '0.0', '0.0', '0.0',      # roll pitch yaw
            'global_map',             # parent
            'robot_1/map',            # child
        ],
    )
    ld.add_action(static_tf_robot1)

    # -------------------------------------------------------
    # Static TF: global_map -> robot_2/map
    # offset of +6.0 m in Y from world file
    # -------------------------------------------------------
    static_tf_robot2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='global_to_robot2_map',
        output='screen',
        arguments=[
            '-0.5', '8.5', '0.0',      # x y z
            '0.0', '0.0', '0.0',      # roll pitch yaw
            'global_map',             # parent
            'robot_2/map',            # child
        ],
    )
    ld.add_action(static_tf_robot2)

    return ld
