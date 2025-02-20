from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_max',
            executable='camera_node',
            name='camera'
        ),
        Node(
            package='vision_max',
            executable='detection_node',
            name='detector'
        ),
        Node(
            package='vision_max',
            executable='predictor_node',
            name='predictor'
        )
    ])