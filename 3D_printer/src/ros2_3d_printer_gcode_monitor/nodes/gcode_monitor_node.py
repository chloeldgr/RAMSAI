import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from 3D_printer_messages.srv import profileCommand
from 3D_printer_messages.srv import gcodeCommand
from 3D_printer_messages.srv import imageCommand


class ProfileCaptureNode(Node):
        def __init__(self):
                super().__init__('3D_printer_profile_capture')


if __name__ == '__main__':
    rclpy.init()
    image_capture_node = ProfileCaptureNode(0)
    rclpy.spin(image_capture_node)
    rclpy.shutdown()