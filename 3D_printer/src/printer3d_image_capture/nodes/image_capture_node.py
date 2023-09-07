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

from printer3d_msgs.srv import ImageCommand
import cv2


class ImageCaptureNode(Node):
    def __init__(self, cam=0):
        super().__init__('printer_image_capture')
        self.cameraNumber = cam
        self.imageService = self.create_service(ImageCommand, 'ask_capture_image', self.takePicture)

    def __del__(self):
        self.webcam.release()

    def takePicture(self, request, response):
        webcam = cv2.VideoCapture("/dev/video"+str(self.cameraNumber))  # Be careful with the number after video
        check, frame = webcam.read()
        webcam.release()
        cv2.imwrite(filename=request.filename, img=frame)
        self.get_logger().info('image acquired : '+request.filename)
        response.confirmation = True
        return response


if __name__ == '__main__':
    rclpy.init()
    image_capture_node = ImageCaptureNode(0)
    rclpy.spin(image_capture_node)
    rclpy.shutdown()
