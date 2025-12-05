import os

import launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory,
)

from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    # =========================================================================
    #  Package paths and basic configuration
    # =========================================================================
    package_name = 'mars_exploration'
    package_dir = get_package_share_directory(package_name)

    world_path = os.path.join(package_dir, 'worlds', 'mars_single_robot.wbt')
    robot_description_path = os.path.join(package_dir, 'resource', 'turtlebot_webots_old.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control_params.yaml')
    slam_params = os.path.join(package_dir, 'resource', 'slam_toolbox_params.yaml')
    nav2_map = os.path.join(package_dir, 'resource', 'mars_map.yaml')
    nav2_params = os.path.join(package_dir, 'resource', 'nav2_params.yaml')

    # TB3 env
    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    # =========================================================================
    #  Launch arguments
    # =========================================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Webots) clock'
    )

    record_rosbag_arg = DeclareLaunchArgument(
        'record_rosbag',
        default_value='false',
        description='Enable rosbag recording (true/false)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    record_rosbag = LaunchConfiguration('record_rosbag')

    # =========================================================================
    #  Webots simulator and robot driver
    # =========================================================================
    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=True,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '/base_link', '/base_footprint'],
    )

    use_twist_stamped = 'ROS_DISTRO' in os.environ and (
        os.environ['ROS_DISTRO'] in ['rolling', 'jazzy', 'kilted']
    )

    if use_twist_stamped:
        mappings = [
            ('/diffdrive_controller/cmd_vel', '/cmd_vel'),
            ('/diffdrive_controller/odom', '/odom'),
        ]
    else:
        mappings = [
            ('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'),
            ('/diffdrive_controller/odom', '/odom'),
        ]

    turtlebot_driver = WebotsController(
        robot_name='robot_1',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': robot_description_path,
                'set_robot_state_publisher': True,
            },
            ros2_control_params,
        ],
        remappings=mappings,
        respawn=True,
    )

    # =========================================================================
    #  ROS 2 Control spawners
    # =========================================================================
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    diffdrive_controller_spawner = Node(
        
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )

    ros_control_spawners = [
        diffdrive_controller_spawner,
        joint_state_broadcaster_spawner,
    ]


    # =========================================================================
    # Nav2 stack nodes
    # ==========================================================================
    nav2_nodes = [
        Node(
            
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params],
            remappings=[('cmd_vel', '/cmd_vel'),
                        ('robot_base_frame', 'base_link')]
        ),
        Node(
            
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params, 
                    {'use_sim_time': use_sim_time}],
        ),
        Node(
            
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}, {'yaml_filename': nav2_map}],
        ),
        Node(
            
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': [
                'map_server', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator'
            ]}],
        ),
    ]

    # =========================================================================
    #  SLAM Toolbox node
    # =========================================================================
    slam_toolbox = Node(
        
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': use_sim_time},
        ]
    )

    # =========================================================================
    #  Mars exploration control node
    # =========================================================================
    mars_exploration_node = Node(
        
        package=package_name,
        executable='mars_exploration_node',
        name='mars_exploration_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {
                'slam_pose_topic': '/slam_toolbox/pose',
                'scan_topic': '/scan',
                'goal_topic': '/goal_pose',
                'cmd_vel_topic': '/cmd_vel',
                'control_rate_hz': 20.0,
            },
        ],
    )

    # =========================================================================
    #  Optional rosbag recorder
    # =========================================================================
    rosbag_recorder = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-a',
            '-x', '/camera/.*',
        ],
        output='screen',
        condition=IfCondition(record_rosbag),
    )

    # =========================================================================
    #  Shutdown behavior
    # =========================================================================
    shutdown_on_webots_exit = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    # =========================================================================
    #  Synchronization – start controllers + slam after driver ready
    # =========================================================================
    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=ros_control_spawners + [slam_toolbox, mars_exploration_node] + nav2_nodes,
    )

    # =========================================================================
    #  Final LaunchDescription
    # =========================================================================
    launch_entities = [
        use_sim_time_arg,
        record_rosbag_arg,

        webots,
        webots._supervisor,

        robot_state_publisher,
        footprint_publisher,

        turtlebot_driver,
        waiting_nodes,

        rosbag_recorder,
        shutdown_on_webots_exit,
    ]

    return LaunchDescription(launch_entities)
