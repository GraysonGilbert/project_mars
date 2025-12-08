import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

from launch_catch_ros2 import Catch2LaunchDescription, Catch2IntegrationTestNode


def generate_launch_description():
    
    # mars_fleet_bringup launch file to start 2 robot demo
    fleet_package = 'mars_fleet_bringup'
    fleet_launch_file = os.path.join(
        get_package_share_directory(fleet_package),
        'launch',
        'mars_fleet_bringup.launch.py'
    )

    # Launch argument to disable rviz from starting
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='false', description='Whether to launch RViz2'
    )

    fleet_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fleet_launch_file),
        launch_arguments={
            'launch_rviz': LaunchConfiguration('launch_rviz')
        }.items()
    )
    
    # Catch 2 Integration test node and launch argument 
    
    catch2_ld = Catch2LaunchDescription()

    test_duration_arg = DeclareLaunchArgument(
        "test_duration",
        default_value="2.0",
        description="Max length of test in seconds."
    )
    integration_test_node = Catch2IntegrationTestNode(
        package="mars_overseer",
        executable="overseer_integration_tests",
        parameters=[{"test_duration": LaunchConfiguration("test_duration")}],
    )
    
    # Delay the integration test node launch by 10 seconds
    integration_test_timer = TimerAction(
        period=10.0,
        actions=[integration_test_node]
    )
    
    # Shutdown processesafter integration test completes
    shutdown_handler = RegisterEventHandler(
    OnProcessExit(
        target_action=integration_test_node,
        on_exit=[EmitEvent(event=Shutdown())],
    )
    )
    
    # build the LaunchDescription
    ld = LaunchDescription()

    ld.add_action(launch_rviz_arg)
    ld.add_action(fleet_launch)
    ld.add_action(catch2_ld)
    ld.add_action(test_duration_arg)
    ld.add_action(integration_test_timer)
    ld.add_action(shutdown_handler)
    return ld
