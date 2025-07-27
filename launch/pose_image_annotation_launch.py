from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_pose_image_annotation',
            executable='pose_publisher',
            name='pose_publisher',
            output='screen'
        ),
        Node(
            package='ros2_pose_image_annotation',
            executable='image_publisher',
            name='image_publisher',
            output='screen'
        ),
        Node(
            package='ros2_pose_image_annotation',
            executable='annotate_node',
            name='annotate_node',
            output='screen'
        ),
    ])
