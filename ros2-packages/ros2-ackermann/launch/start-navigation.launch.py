import launch
from launch.substitutions import Command
import launch_ros
import os

def generate_launch_description():
    my_pkg_share = launch_ros.substitutions.FindPackageShare(package='ros2-ackermann').find('ros2-ackermann')
    nav2_pkg_share = launch_ros.substitutions.FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    model_path = os.path.join(my_pkg_share, 'src/description/ackermann_description.urdf')
    rviz_config_path = os.path.join(my_pkg_share, 'config/rviz.config.rviz')
    slam_config_path = os.path.join(my_pkg_share, 'config/slam_toolbox.config.yaml')
    nav2_launch_path = os.path.join(nav2_pkg_share, 'launch/navigation_launch.py')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', model_path])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    ackermann_node = launch_ros.actions.Node(
        package='ros2-ackermann',
        executable='ackermann_tf2_broadcaster',
        name='broadcaster_ackermann',
        parameters=[
            {'odom_topic_name': 'odom'},
            {'odom_frame_id': 'odom'},
            {'child_frame_id': 'base_link'}
        ]
    )
    tf2_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'odom']
    )
    slam_node = launch_ros.actions.Node(
        parameters=[
          slam_config_path,
          {'use_sim_time': False}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox'
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )
    navigation_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            nav2_launch_path
        )
    )

    return launch.LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        ackermann_node,
        tf2_node,
        slam_node,
        navigation_launch,
        rviz_node
    ])