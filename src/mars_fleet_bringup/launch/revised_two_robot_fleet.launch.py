import os

import launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_name = 'mars_exploration'
    package_dir = get_package_share_directory(package_name)

    # New world with TWO robots in it
    world_path = os.path.join(package_dir, 'worlds', 'mars_two_robots.wbt')

    robot_instance_launch = os.path.join(
        package_dir,
        'launch',
        'robot_instance.launch.py',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Webots) clock',
    )

    record_rosbag_arg = DeclareLaunchArgument(
        'record_rosbag',
        default_value='false',
        description='Enable rosbag recording (true/false)',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    record_rosbag = LaunchConfiguration('record_rosbag')

    # Webots simulator
    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=True,
    )

    # Two robot instances
    robot_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_instance_launch),
        launch_arguments={
            'robot_id': 'robot_1',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    robot_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_instance_launch),
        launch_arguments={
            'robot_id': 'robot_2',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Optional rosbag (global)
    rosbag_recorder = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-a',
            '-x', '/camera/.*',
        ],
        output='screen',
        condition=launch.conditions.IfCondition(record_rosbag),
    )

    # Shutdown when Webots exits
    shutdown_on_webots_exit = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        record_rosbag_arg,
        webots,
        webots._supervisor,
        robot_1,
        robot_2,
        rosbag_recorder,
        shutdown_on_webots_exit,
    ])
