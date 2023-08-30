from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='printer3d_driver',
            namespace='printer_gcode_sender',
            executable='gcode_monitor_node.py',
            name='gcode_monitor_node'
        ),
        Node(
            package='printer3d_image_capture',
            namespace='printer_image_capture',
            executable='image_capture_node.py',
            name='image_capture_node'
        ),
        Node(
            package='printer3d_profile_capture',
            namespace='printer_gocator_communication',
            executable='gocator_sensor_node.py',
            name='gocator_sensor_node'
        ),
        Node(
            package='printer3d_manager',
            namespace='printer_manager',
            executable='printer_scanner_imageCapture_node.py',
            name='printer_control_node'
        )
    ])