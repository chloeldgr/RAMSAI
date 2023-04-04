#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ros2_3d_printer_messages.srv import ProfileCommand
from ros2_3d_printer_messages.srv import GcodeCommand
from ros2_3d_printer_messages.srv import ImageCommand


class PrinterControlNode(Node):
        def __init__(self):
                super().__init__('printer_control')
                self.client_image_capture = self.create_client(ImageCommand, 'ask_capture_image')
                while not self.client_image_capture.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info('image capture service not available, waiting again...')
                self.req_image_capture = ImageCommand.Request()

        def sendImageCaptureRequest(self,filename):
                self.req_image_capture.filename = filename
                self.future_image_capture = self.client_image_capture.call_async(self.req_image_capture)
                while rclpy.ok():
                        rclpy.spin_once(self)
                        if self.future_image_capture.done():
                                self.get_logger().info('capture finished')
                                break



if __name__ == '__main__':
    rclpy.init()
    printer_control_node = PrinterControlNode()
    for i in range(0,10):
        printer_control_node.sendImageCaptureRequest('impressions/test/image_test_'+str(i))
    rclpy.shutdown()