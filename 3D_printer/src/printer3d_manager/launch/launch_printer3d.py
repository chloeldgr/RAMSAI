from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='printer3d_manager',
            namespace='printer1',
            executable='printer_control_node',
            name='printer_control_node'
        ),
        Node(
            package='printer3d_image_capture',
            namespace='printer1',
            executable='image_capture_node',
            name='image_capture_node'
        ),
        Node(
            package='printer3d_profile_capture',
            namespace='printer1',
            executable='gocatorSensorNode',
            name='profile_capture_node'
        )
    ])
