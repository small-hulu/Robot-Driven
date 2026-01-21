from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='cpp_pkg',
            executable='driver_node',
            name='turn_on_robot',
            output='screen',
            parameters=[],
        )

        Node(
            package='cpp_pkg',
            executable='driver_node', 
            name='data_publisher',
            output='screen',
            parameters=[],
        )


    ])
