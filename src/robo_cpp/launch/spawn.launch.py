import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'robo_cpp'
    pkg_share = FindPackageShare(pkg_name)

    # Path to xacro file
    xacro_file = PathJoinSubstitution([
        pkg_share, 'urdf', 'robo_cpp.urdf.xacro'
    ])

    # Generate URDF from Xacro
    robot_description = Command(['xacro ', xacro_file])

    # Path to world
    world_path = PathJoinSubstitution([
        pkg_share, 'worlds', 'wall_world.world'
    ])

    # Path to RViz config
    rviz_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'view_config.rviz'
    )

    return LaunchDescription([
        # Launch Gazebo with custom world
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                world_path,
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        # Joint State Publisher (for simulating joints)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Spawn the robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robo_bot',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.5'
            ],
            output='screen'
        ),

        # Launch RViz with config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
    ]
)
