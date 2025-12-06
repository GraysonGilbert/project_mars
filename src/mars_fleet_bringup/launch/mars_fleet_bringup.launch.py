import os

import launch
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch.actions import OpaqueFunction


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


def generate_launch_description():
    fleet_package = 'mars_fleet_bringup'
    #robot_package = 'mars_exploration'
    robot_launch_file = os.path.join(
        get_package_share_directory(fleet_package),
        'launch',
        'robot_instance.launch.py',
    )

    package_dir = get_package_share_directory(fleet_package)
    world_path = os.path.join(package_dir, 'worlds', 'mars_2_robots.wbt')

    # Launch arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots', default_value='2', description='Number of robots to launch'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Webots) clock'
    )
    record_rosbag_arg = DeclareLaunchArgument(
        'record_rosbag', default_value='false', description='Enable rosbag recording (true/false)'
    )

    record_rosbag = LaunchConfiguration('record_rosbag')

    # Webots simulator
    webots = WebotsLauncher(world=world_path, ros2_supervisor=True)
   
    # Optional global rosbag recording
    rosbag_recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-x', '/camera/.*'],
        output='screen',
        condition=IfCondition(record_rosbag),
    )

    # Shutdown fleet when Webots exits
    shutdown_on_webots_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=webots,
            on_exit=[EmitEvent(event=launch.events.Shutdown())],
        )
    )    
       
    ld = LaunchDescription([num_robots_arg, use_sim_time_arg, record_rosbag_arg])   
    
    ld.add_action(webots)
    ld.add_action(webots._supervisor)
    ld.add_action(OpaqueFunction(function=launch_robots, kwargs={'robot_launch_file': robot_launch_file}))
    ld.add_action(rosbag_recorder)
    ld.add_action(shutdown_on_webots_exit)

    return ld
