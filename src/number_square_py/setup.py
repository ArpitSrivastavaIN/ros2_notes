from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Path to Xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare("robo_cpp"),
        "urdf",
        "robo_cpp.urdf.xacro"
    ])

    # Convert Xacro to robot_description
    robot_description = ParameterValue(
        Command(["xacro", xacro_file]),
        value_type=str
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen"
    )

    # Launch Gazebo with custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("gazebo_ros"),
            "launch",
            "gazebo.launch.py"
        ])),
        launch_arguments={
            "world": PathJoinSubstitution([
                FindPackageShare("robo_cpp"),
                "worlds",
                "wall_world.world"
            ])
        }.items()
    )

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "robo_bot"
        ],
        output="screen"
    )

    # Teleop node (opens in new terminal using xterm)
    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_keyboard",
        prefix="xterm -e",  # Launches in separate terminal window
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        teleop
    ])
