import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    EmitEvent,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_name = 'mars_fleet_bringup'
    package_dir = get_package_share_directory(package_name)

    robot_package_name = 'mars_exploration'
    robot_package_dir = get_package_share_directory(robot_package_name)

    # Hard-coded default multi-robot world path (string, not LaunchConfiguration)
    world_path = os.path.join(package_dir, 'worlds', 'mars_two_robot.wbt')

    # =========================================================================
    #  Launch arguments
    # =========================================================================
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

    # =========================================================================
    #  Webots simulator
    # =========================================================================
    webots = WebotsLauncher(
        world=world_path,        # <-- plain string path now
        ros2_supervisor=True,
    )

    # Shut down ROS when Webots exits
    shutdown_on_webots_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=webots,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    # =========================================================================
    #  Robot bringup includes (robot_1, robot_2)
    # =========================================================================
    robot_bringup_launch = os.path.join(
        robot_package_dir, 'launch', 'robot_bringup.launch.py'
    )

    robots_bringup = []
    
    for robot_id in ['robot_1']:
        robots_bringup.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_bringup_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'robot_name': robot_id,
            }.items(),
        ))

    # =========================================================================
    #  Optional rosbag recorder (records everything)
    # =========================================================================
    rosbag_recorder = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-a',
            '-x', '/camera/.*',
        ],
        output='screen',
        condition=IfCondition(record_rosbag),   # <-- needs IfCondition wrapper
    )

    # =========================================================================
    #  Final LaunchDescription
    # =========================================================================
    return LaunchDescription([
        use_sim_time_arg,
        record_rosbag_arg,

        webots,
        webots._supervisor,

        rosbag_recorder,
        shutdown_on_webots_exit,
    ] + robots_bringup)
