from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the map to image publisher
        Node(
            package='ros2_pose_image_annotation',
            executable='map_to_image_publisher',
            name='map_to_image_publisher',
            output='screen'
        ),
        # Start the filter node
        Node(
            package='ros2_pose_image_annotation',
            executable='gz_pose_filter',
            name='gz_pose_filter',
            output='screen'
        ),
        # Start the annotate node
        Node(
            package='ros2_pose_image_annotation',
            executable='annotate_node',
            name='annotate_node',
            output='screen'
        ),
        # Start foxglove_bridge for WebSocket on port 9090
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{'port': 9090}]
        )
    ])
