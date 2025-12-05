import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_name = 'mars_exploration'
    package_dir = get_package_share_directory(package_name)

    robot_description_path = os.path.join(package_dir, 'resource', 'turtlebot_webots.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control_params.yaml')
    slam_params = os.path.join(package_dir, 'resource', 'slam_toolbox_params.yaml')

    # NEW: Nav2 config (reuse the same files as single-robot example)
    nav2_map = os.path.join(package_dir, 'resource', 'mars_map.yaml')
    nav2_params = os.path.join(package_dir, 'resource', 'nav2_params.yaml')

    # TB3 env
    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    # Launch arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='robot_1',
        description='Robot namespace / Webots robot name',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time',
    )

    def launch_setup(context, *args, **kwargs):
        # Resolve substitutions to plain Python values
        robot_id = LaunchConfiguration('robot_id').perform(context)
        use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
        use_sim_time = (use_sim_time_str.lower() == 'true')
        nav2_params = os.path.join(package_dir, 'resource', f'{robot_id}_nav2_params.yaml')

        # ---------------------------------------------------------------------
        # Static TF: from robot_i/base_link -> robot_i/LDS-01
        # (LaserScan comes in as frame_id="LDS-01"; we keep that but attach it
        # under the namespaced TF tree via robot_id/ frame prefix.)
        # ---------------------------------------------------------------------
        static_laser_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{robot_id}_laser_tf',
            namespace=robot_id,
            # x y z roll pitch yaw parent child
            arguments=[
                '0.0', '0.0', '0.1',              # 10cm above base_link
                '0.0', '0.0', '0.0',
                f'{robot_id}/base_link',          # parent frame in TF tree
                f'{robot_id}/LDS-01',             # child frame – scan frame
            ],
        )

        footprint_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            namespace=robot_id,
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                f'{robot_id}/base_link',
                f'{robot_id}/base_footprint'],
        )

        # Robot state publisher, with a frame_prefix per robot
        robot_state_publisher = Node(
            namespace=robot_id,
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': '<robot name=""><link name=""/></robot>',
                'frame_prefix': f'{robot_id}/',
            }],
        )

        # Topic remappings relative to namespace
        use_twist_stamped = 'ROS_DISTRO' in os.environ and (
            os.environ['ROS_DISTRO'] in ['rolling', 'jazzy', 'kilted']
        )

        if use_twist_stamped:
            mappings = [
                ('diffdrive_controller/cmd_vel', 'cmd_vel'),
                ('diffdrive_controller/odom', 'odom'),
            ]
        else:
            mappings = [
                ('diffdrive_controller/cmd_vel_unstamped', 'cmd_vel'),
                ('diffdrive_controller/odom', 'odom'),
            ]

        # Webots driver for this robot
        turtlebot_driver = WebotsController(
            robot_name=robot_id,          # Webots robot name
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
            namespace=robot_id,
        )

        # ---------------------------------------------------------------------
        # ROS 2 Control spawners (namespaced)
        # ---------------------------------------------------------------------
        controller_manager_timeout = ['--controller-manager-timeout', '50']
        controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

        diffdrive_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            prefix=controller_manager_prefix,
            arguments=['diffdrive_controller'] + controller_manager_timeout,
            namespace=robot_id,
        )

        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            prefix=controller_manager_prefix,
            arguments=['joint_state_broadcaster'] + controller_manager_timeout,
            namespace=robot_id,
        )

        ros_control_spawners = [
            diffdrive_controller_spawner,
            joint_state_broadcaster_spawner,
        ]

        # ---------------------------------------------------------------------
        # SLAM toolbox (namespaced, using corrected scan)
        # ---------------------------------------------------------------------
        slam_toolbox = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=robot_id,
            output='screen',
            parameters=[
                slam_params,
                {
                    'use_sim_time': use_sim_time,
                    'map_frame': f'{robot_id}/map',
                    'odom_frame': f'{robot_id}/odom',
                    'base_frame': f'{robot_id}/base_link',
                }
            ],
            remappings=[
                ('scan', 'scan_corrected'),   # /robot_i/scan_corrected
            ],
        )

        # ---------------------------------------------------------------------
        # Scan frame rewriter (namespaced)
        # ---------------------------------------------------------------------
        scan_frame_rewriter = Node(
            package='mars_exploration',
            executable='scan_frame_rewriter',
            name='scan_frame_rewriter',
            namespace=robot_id,
            parameters=[
                {
                    # These are relative to the namespace:
                    #   /robot_i/scan           -> input
                    #   /robot_i/scan_corrected -> output
                    'input_topic': 'scan',
                    'output_topic': 'scan_corrected',
                    'new_frame_id': f'{robot_id}/LDS-01',
                }
            ],
        )

        # ---------------------------------------------------------------------
        # Nav2 stack (one instance per robot, namespaced)
        # ---------------------------------------------------------------------
        # We override frames so each robot has its own TF tree:
        #   robot_i/map -> robot_i/odom -> robot_i/base_link
        # and all nav2 topics live under /robot_i/...
        # nav2_common_frame_overrides = {
        #     'use_sim_time': use_sim_time,
        #     'global_frame': f'{robot_id}/map',
        #     'robot_base_frame': f'{robot_id}/base_link',
        #     'odom_frame': f'{robot_id}/odom',

        #      # controller_server local costmap
        #     'local_costmap.global_frame': f'{robot_id}/odom',
        #     'local_costmap.robot_base_frame': f'{robot_id}/base_link',

        #     # planner_server global costmap
        #     'global_costmap.global_frame': f'{robot_id}/map',
        #     'global_costmap.robot_base_frame': f'{robot_id}/base_link',

        #     # core controller config
        #     'controller_frequency': 10.0,
        #     'failure_tolerance': 0.3,
        #     'progress_checker_plugin': 'progress_checker',
        #     'goal_checker_plugins': ['goal_checker'],
        #     'controller_plugins': ['FollowPath'],

        #     # FollowPath (DWB) core limits
        #     'FollowPath.plugin': 'dwb_core::DWBLocalPlanner',
        #     'FollowPath.min_vel_x': 0.0,
        #     'FollowPath.max_vel_x': 0.22,
        #     'FollowPath.min_speed_xy': 0.0,
        #     'FollowPath.max_speed_xy': 0.22,
        #     'FollowPath.min_vel_y': 0.0,
        #     'FollowPath.max_vel_y': 0.0,
        #     'FollowPath.max_vel_theta': 1.0,
        #     'FollowPath.min_speed_theta': 0.0,

        #     'FollowPath.acc_lim_x': 2.5,
        #     'FollowPath.decel_lim_x': -2.5,
        #     'FollowPath.acc_lim_y': 0.0,
        #     'FollowPath.decel_lim_y': 0.0,
        #     'FollowPath.acc_lim_theta': 3.2,
        #     'FollowPath.decel_lim_theta': -3.2,

        #     'FollowPath.vx_samples': 20,
        #     'FollowPath.vy_samples': 0,
        #     'FollowPath.vtheta_samples': 40,
        #     'FollowPath.sim_time': 1.5,
        #     'FollowPath.linear_granularity': 0.05,
        #     'FollowPath.angular_granularity': 0.025,
        #     'FollowPath.transform_tolerance': 1.0,

        #     # *** THE IMPORTANT ONE ***
        #     'FollowPath.critics': [
        #         'RotateToGoal',
        #         'Oscillation',
        #         'BaseObstacle',
        #         'GoalAlign',
        #         'PathAlign',
        #         'PathDist',
        #         'GoalDist',
        #     ],

        #     # Some critic weights (optional but nice)
        #     'FollowPath.BaseObstacle.scale': 0.02,
        #     'FollowPath.PathAlign.scale': 32.0,
        #     'FollowPath.PathAlign.forward_point_distance': 0.1,
        #     'FollowPath.GoalAlign.scale': 24.0,
        #     'FollowPath.GoalAlign.forward_point_distance': 0.1,
        #     'FollowPath.PathDist.scale': 32.0,
        #     'FollowPath.GoalDist.scale': 24.0,
        #     'FollowPath.RotateToGoal.scale': 32.0,
        #     'FollowPath.RotateToGoal.slowing_factor': 5.0,
        #     'FollowPath.RotateToGoal.lookahead_time': -1.0,
        # }

        # nav2_param_substitutions = {
        #     'global_frame': f'{robot_id}/map',
        #     'robot_base_frame': f'{robot_id}/base_link',
        #     'odom_frame': f'{robot_id}/odom',
        #     'local_costmap.global_frame': f'{robot_id}/odom',
        #     'local_costmap.robot_base_frame': f'{robot_id}/base_link',
        #     'global_costmap.global_frame': f'{robot_id}/map',
        #     'global_costmap.robot_base_frame': f'{robot_id}/base_link',
        # }

        # configured_nav2_params = RewrittenYaml(
        #     source_file=nav2_params,
        #     root_key='',
        #     param_rewrites=nav2_param_substitutions,
        #     convert_types=True,
        # )

        nav2_nodes = [
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                namespace=robot_id,
                output='screen',
                parameters=[
                    nav2_params,
                    {
                        'use_sim_time': use_sim_time,
                    }
                ],
                # cmd_vel resolves to /robot_i/cmd_vel automatically
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                namespace=robot_id,
                output='screen',
                parameters=[
                    nav2_params,
                    {
                        'use_sim_time': use_sim_time,
                    }
                ],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                namespace=robot_id,
                output='screen',
                parameters=[
                    nav2_params,
                    {
                        'use_sim_time': use_sim_time,
                    }
                ],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                namespace=robot_id,
                output='screen',
                parameters=[
                    nav2_params,
                    {
                        'use_sim_time': use_sim_time,
                    }
                ],
            ),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                namespace=robot_id,
                output='screen',
                parameters=[
                    nav2_params,
                    {
                        'use_sim_time': use_sim_time,
                        'yaml_filename': nav2_map,
                        'frame_id': f'{robot_id}/map',
                    },
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                namespace=robot_id,
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    # Names are relative to this namespace, so this
                    # manages /robot_i/map_server, /robot_i/planner_server, ...
                    'node_names': [
                        'map_server',
                        'planner_server',
                        'controller_server',
                        'behavior_server',
                        'bt_navigator',
                    ],
                }],
            ),
        ]

        # ---------------------------------------------------------------------
        # Mars exploration node (namespaced, talking to nav2 NavigateToPose)
        # ---------------------------------------------------------------------
        mars_exploration_node = Node(
            package=package_name,
            executable='mars_exploration_node',
            name='mars_exploration_node',
            namespace=robot_id,
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {
                    # Topics are relative to namespace:
                    #   /robot_i/slam_toolbox/pose (if your node builds that path)
                    #   /robot_i/scan
                    'slam_pose_topic': 'pose',
                    'scan_topic': 'scan',
                    'goal_topic': 'goal_pose',
                    'cmd_vel_topic': 'cmd_vel',
                    'map_topic': 'map',

                    # Frames line up with nav2 + slam
                    'global_frame': f'{robot_id}/map',
                    'base_frame':   f'{robot_id}/base_link',

                    'control_rate_hz': 20.0,
                },
            ],
        )

        # ---------------------------------------------------------------------
        # Synchronization – start controllers + SLAM + Nav2 + exploration
        # only after the Webots driver is connected
        # ---------------------------------------------------------------------
        waiting_nodes = WaitForControllerConnection(
            target_driver=turtlebot_driver,
            nodes_to_start=ros_control_spawners
                           + nav2_nodes
                           + [slam_toolbox, mars_exploration_node],
        )

        # Return the actions to add to the LaunchDescription
        return [
            turtlebot_driver,
            waiting_nodes,
            static_laser_tf,
            robot_state_publisher,
            scan_frame_rewriter,
            footprint_publisher,
        ]

    return LaunchDescription([
        robot_id_arg,
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup),
    ])
