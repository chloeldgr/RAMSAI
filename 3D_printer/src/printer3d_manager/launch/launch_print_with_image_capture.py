from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('printer3d_manager'),
        'config',
        'params.yaml'
        )
        
    with open(config, 'r') as f:
        configYaml = yaml.safe_load(f)
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
            package='printer3d_manager',
            namespace='printer_manager',
            executable='printer_imageCapture_node.py',
            name='printer_control_node',
            parameters = [configYaml['printer3d_manager']['ros__parameters']]
        )
    ])