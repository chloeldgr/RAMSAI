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
            package='printer3d_manager',
            namespace='printer_manager',
            executable='printer_node.py',
            name='printer_control_node'
        )
    ])