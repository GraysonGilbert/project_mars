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
from launch.actions import OpaqueFunction


def _launch_setup(context, *args, **kwargs):
    """
    This function runs inside an OpaqueFunction so we can resolve
    LaunchConfiguration values (especially robot_name) to plain strings.
    """

    # =========================================================================
    #  Package paths and basic configuration
    # =========================================================================
    package_name = 'mars_exploration'
    package_dir = get_package_share_directory(package_name)

    # (world_path is unused here; Webots is started in the fleet launch file)
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control_params.yaml')
    slam_params = os.path.join(package_dir, 'resource', 'slam_toolbox_params.yaml')
    nav2_map = os.path.join(package_dir, 'resource', 'mars_map.yaml')
    nav2_params = os.path.join(package_dir, 'resource', 'nav2_params.yaml')
    robot_description_path = os.path.join(package_dir, 'resource', 'turtlebot_webots.urdf')

    # TB3 env
    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    # =========================================================================
    #  Resolve launch configurations
    # =========================================================================
    use_sim_time_lc = LaunchConfiguration('use_sim_time')
    robot_name_lc = LaunchConfiguration('robot_name')

    # Key fix: resolve robot_name to a plain string for WebotsController
    robot_name = robot_name_lc.perform(context)

    # (We can keep use_sim_time as a substitution; Node parameters understand it)
    use_sim_time = use_sim_time_lc

    # =========================================================================
    #  Webots robot driver & basic TF
    # =========================================================================
    # with open(robot_description_path, 'r') as f:
    #     robot_description_content = f.read()
    
    # robot_state_publisher = Node(
    #     namespace=robot_name,
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'robot_description': robot_description_content,
    #         'frame_prefix': f'{robot_name}/',
    #     }],
    # )

    robot_state_publisher = Node(
        namespace=robot_name,
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>',
            'frame_prefix': f'{robot_name}/',
        }],
    )

    footprint_publisher = Node(
        namespace=robot_name,
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', f'/{robot_name}/base_link', f'/{robot_name}/base_footprint'],
    )

    use_twist_stamped = 'ROS_DISTRO' in os.environ and (
        os.environ['ROS_DISTRO'] in ['rolling', 'jazzy', 'kilted']
    )

    if use_twist_stamped:
        mappings = [
            ('/diffdrive_controller/cmd_vel', 'cmd_vel'),
            ('/diffdrive_controller/odom', 'odom'),
        ]
    else:
        mappings = [
            ('/diffdrive_controller/cmd_vel_unstamped', 'cmd_vel'),
            ('/diffdrive_controller/odom', 'odom'),
        ]

    # --------- HERE IS THE IMPORTANT CHANGE ----------
    turtlebot_driver = WebotsController(
        namespace=robot_name,      # plain string
        robot_name=robot_name,     # plain string, NOT LaunchConfiguration
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
    # --------------------------------------------------

    # =========================================================================
    #  ROS 2 Control spawners
    # =========================================================================
    controller_manager_timeout = ['--controller-manager-timeout', '500']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    controller_manager_name = f'/{robot_name}/controller_manager'

    joint_state_broadcaster_spawner = Node(
        namespace=robot_name,
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', controller_manager_name,
        ] + controller_manager_timeout,
    )

    diffdrive_controller_spawner = Node(
        namespace=robot_name,
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=[
            'diffdrive_controller',
            "--controller-manager", controller_manager_name,
        ] + controller_manager_timeout,
    )

    ros_control_spawners = [
        joint_state_broadcaster_spawner,
        diffdrive_controller_spawner,
    ]

    # =========================================================================
    # Nav2 stack nodes
    # (unchanged; you can later decide if you want them namespaced per robot)
    # ==========================================================================
    nav2_frame_params = {
        'use_sim_time': use_sim_time,
        'global_frame': f'{robot_name}/map',
        'robot_base_frame': f'{robot_name}/base_link',
        'odom_frame': f'{robot_name}/odom',
    }

    controller_extra_params = {
        # core controller config
        'controller_frequency': 10.0,
        'failure_tolerance': 0.3,
        'progress_checker_plugin': 'progress_checker',
        'goal_checker_plugins': ['goal_checker'],
        'controller_plugins': ['FollowPath'],

        # FollowPath (DWB) core limits
        'FollowPath.plugin': 'dwb_core::DWBLocalPlanner',
        'FollowPath.min_vel_x': 0.0,
        'FollowPath.max_vel_x': 0.22,
        'FollowPath.min_speed_xy': 0.0,
        'FollowPath.max_speed_xy': 0.22,
        'FollowPath.min_vel_y': 0.0,
        'FollowPath.max_vel_y': 0.0,
        'FollowPath.max_vel_theta': 1.0,
        'FollowPath.min_speed_theta': 0.0,

        'FollowPath.acc_lim_x': 2.5,
        'FollowPath.decel_lim_x': -2.5,
        'FollowPath.acc_lim_y': 0.0,
        'FollowPath.decel_lim_y': 0.0,
        'FollowPath.acc_lim_theta': 3.2,
        'FollowPath.decel_lim_theta': -3.2,

        'FollowPath.vx_samples': 20,
        'FollowPath.vy_samples': 0,
        'FollowPath.vtheta_samples': 40,
        'FollowPath.sim_time': 1.5,
        'FollowPath.linear_granularity': 0.05,
        'FollowPath.angular_granularity': 0.025,
        'FollowPath.transform_tolerance': 1.0,

        # *** THE IMPORTANT ONE ***
        'FollowPath.critics': [
            'RotateToGoal',
            'Oscillation',
            'BaseObstacle',
            'GoalAlign',
            'PathAlign',
            'PathDist',
            'GoalDist',
        ],

        # Some critic weights (optional but nice)
        'FollowPath.BaseObstacle.scale': 0.02,
        'FollowPath.PathAlign.scale': 32.0,
        'FollowPath.PathAlign.forward_point_distance': 0.1,
        'FollowPath.GoalAlign.scale': 24.0,
        'FollowPath.GoalAlign.forward_point_distance': 0.1,
        'FollowPath.PathDist.scale': 32.0,
        'FollowPath.GoalDist.scale': 24.0,
        'FollowPath.RotateToGoal.scale': 32.0,
        'FollowPath.RotateToGoal.slowing_factor': 5.0,
        'FollowPath.RotateToGoal.lookahead_time': -1.0,
    }

    nav2_nodes = [
        Node(
            namespace=robot_name,
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params, nav2_frame_params],
            remappings=[('cmd_vel', 'cmd_vel')]
        ),
        Node(
            namespace=robot_name,
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params, nav2_frame_params, controller_extra_params],
        ),
        Node(
            namespace=robot_name,
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[nav2_params, nav2_frame_params],
        ),
        Node(
            namespace=robot_name,
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[nav2_params, nav2_frame_params],
        ),
        Node(
            namespace=robot_name,
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[nav2_params,
                        nav2_frame_params,
                        {'yaml_filename': nav2_map}],
        ),
        Node(
            namespace=robot_name,
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
        namespace=robot_name,
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {
                'use_sim_time': use_sim_time,
                'map_frame': f'{robot_name}/map',
                'odom_frame': f'{robot_name}/odom',
                'base_frame': f'{robot_name}/base_footprint',
            },
        ]
    )

    # =========================================================================
    #  Mars exploration control node
    # =========================================================================
    mars_exploration_node = Node(
        namespace=robot_name,
        package=package_name,
        executable='mars_exploration_node',
        name='mars_exploration_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {
                'slam_pose_topic': 'slam_toolbox/pose',
                'scan_topic': 'scan',
                'goal_topic': 'goal_pose',
                'cmd_vel_topic': 'cmd_vel',
                'map_topic': f'{robot_name}/map',
                'global_frame': f'{robot_name}/map',
                'base_frame': f'{robot_name}/base_link',
                'control_rate_hz': 5.0,
            },
        ],
    )

    # =========================================================================
    #  Synchronization – start controllers + slam after driver ready
    # =========================================================================
    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=ros_control_spawners + [slam_toolbox, mars_exploration_node] + nav2_nodes,
    )

    # Return all entities created in this setup function
    return [
        robot_state_publisher,
        footprint_publisher,
        turtlebot_driver,
        waiting_nodes,
    ]


def generate_launch_description():
    # =========================================================================
    #  Launch arguments
    # =========================================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Webots) clock'
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='TurtleBot3Burger',
        description='Name of the Webots robot to attach to',
    )

    # We only declare arguments here and let OpaqueFunction build everything
    # after launch configurations are known.
    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,
        OpaqueFunction(function=_launch_setup),
    ])
