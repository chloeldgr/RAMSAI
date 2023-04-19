#!/usr/bin/env python3
# Copyright 2023 ICube Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from printer3d_msgs.srv import GcodeCommand
import cv2

YPOSPHOTO = 200
"""
        Imagecapturenode class constructor.

        Parameters
        ----------
            cam : int
                camera number and specifically designed for linux systems. Defaults to 0.

        """

class ImageCaptureNode(Node):
    def __init__(self, cam=0):
        """
        Imagecapturenode class constructor.

        Args:
        ----
            cam (int, optional): _description_. Defaults to 0.

        """
        super().__init__('printer_image_capture')
        self.cameraNumber = cam
        self.client_printer_driver = self.create_client(GcodeCommand, 'send_gcode')
        while not self.client_printer_driver.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('profile measure service not available, waiting again...')
        self.req_printer_driver = GcodeCommand.Request()

    def __del__(self):
        """Imagecapturenode destructor. Release the camera."""
        self.webcam.release()

    def showVideo(self):
        """
        Show the image captured by the camera continuously. Can be stopped by pressing q.

        Returns
        -------
            int: returns 0

        """
        self.webcam = cv2.VideoCapture("/dev/video" + str(self.cameraNumber))  # Be careful with the number after video
        while (True):
            check, frame = self.webcam.read()
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.webcam.release()
        cv2.destroyAllWindows()
        return 0

    def sendGcodeSendingRequest(self, gcode):
        """
        Ask for the sending of the gcode line "gcode" through the ROS2 service "send_gcode".

        Args:
        ----
            gcode: list of gcode lines

        """
        self.req_printer_driver.gcode_strings = gcode
        self.future_printer_driver = self.client_printer_driver.call_async(self.req_printer_driver)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future_printer_driver.done():
                self.get_logger().info('gcode sended')
                break


if __name__ == '__main__':
    rclpy.init()
    image_capture_node = ImageCaptureNode(2)
    gcode = ['G28\n', 'G1 Y'+str(YPOSPHOTO)+' F2400\n']
    image_capture_node.sendGcodeSendingRequest(gcode)
    image_capture_node.showVideo()
    rclpy.shutdown()
