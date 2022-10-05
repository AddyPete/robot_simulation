import os
import pathlib
from pickle import TRUE
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher


package_name = 'robot_simulation'

def generate_launch_description():
    package_dir = get_package_share_directory(package_name)
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'Pioneer_3-DX.urdf')).read_text()
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yml')
    robot_localization_file_path = pathlib.Path(os.path.join(package_dir, 'config', 'ekf.yaml'))
    rviz_config_file = os.path.join(package_dir, 'config', 'rviz_config.rviz')
    slam_toolbox_params = os.path.join(package_dir, 'config', 'slam_config.yaml')
	# remappings=[('odometry/filtered','odom')]
    use_sim_time = True

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'adeptNav2.wbt')
    )

    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    use_deprecated_spawner_py = 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] == 'foxy'

    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )

    mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel')]
    if 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] in ['humble', 'rolling']:
        mappings.append(('/diffdrive_controller/odom', '/odom'))

    adeptnav2 = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[
            {'robot_description': robot_description,
                'use_sim_time': use_sim_time,
                'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>', 
        }],#remappings=remappings
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # start Robot Localization Package
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        output='screen',
        parameters=[robot_localization_file_path]
    )
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    #Hello test
    ekf_estimator = Node(
        package='robot_localization',
        executable='ekf_node',
        output='screen',
        parameters=[{
            'frequency': 10.0,
            'two_d_mode': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',
            'imu0': '/imu',
            'imu0_config': [False, False, False,
                            False, False, True,
                            False, False, False,
                            False, False, True,
                            False, False, False],
            'imu0_queue_size': 40,
            #'odom0': '/odom0',
            'odom0': '/odom',
            'odom0_config': [True, True, False,
                                False, False, True,
                                True, False, False,
                                False, False, True,
                                False, False, False],
            'odom0_queue_size': 40,
            'imu0_differential': False,
            'publish_tf': True,
            'use_control': False,
            'control_config': [True, False, False,
                                False, False, True],
            'debug': False,
        }],
        #remappings=[('/odometry/filtered', '/odom')]
    )		
    odom_estimator = Node(
        package=package_name,
        executable='odom_estimator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )	
    slam_toolbox = Node(
        #parameters=[{'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        #condition=launch.conditions.IfCondition(use_slam)
        parameters=[
            {'use_sim_time': use_sim_time,
                'params_file': slam_toolbox_params},
        ],
    )	
    return LaunchDescription([
        webots,
        adeptnav2,
        robot_state_publisher,
        footprint_publisher,
        #start_robot_localization_cmd,
        #odom_estimator,
        ekf_estimator,
        rviz_node,
        slam_toolbox,
        joint_state_broadcaster_spawner,
        diffdrive_controller_spawner,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])