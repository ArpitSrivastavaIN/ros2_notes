# Importing necessary dependencies for setup launch file
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Launch for number_publisher
        Node(
            package="number_square_py", # Package name
            executable = "number_publisher", # Executable name
            name = "number_publisher", # Node name
            output = "screen" # Tells console output to be printed onto the terminal
        ),

        #Launch for number subscriber
        Node(
            package="number_square_py",
            executable = "square_subscriber",
            name = "square_subscriber",
            output = "screen"
        )

    ])