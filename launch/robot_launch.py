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
    urdf_file_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    robot_description = pathlib.Path(urdf_file_path).read_text()
    use_sim_time = True

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )

    my_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True
             },
        ],
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
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )    
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
            'odom0': '/odom0',
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
        remappings=[('/odometry/filtered', '/odom')]
    )
    odom_estimator = Node(
        package=package_name,
        executable='odom_estimator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )
    cartographer_config_dir = os.path.join(package_dir, 'config')
    configuration_basename = 'my_robot_lds_2d.lua'
    resolution = '0.05'
    publish_period_sec = '1.0'
    rviz_config_file = os.path.join(package_dir, 'config', 'rviz_config.rviz')

    cartographer_node = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename])
    cartographer_occupency_grid_node = Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    slam_toolbox = Node(
        parameters=[{'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        #condition=launch.conditions.IfCondition(use_slam)
    )
    return LaunchDescription([
        webots,
        my_robot_driver,
        odom_estimator,
        robot_state_publisher,
        ekf_estimator,
        #cartographer_node,
        #cartographer_occupency_grid_node,
        footprint_publisher,
        rviz_node,
        slam_toolbox,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
