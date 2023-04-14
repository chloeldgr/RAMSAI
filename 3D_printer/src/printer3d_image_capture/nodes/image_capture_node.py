#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from printer3d_msgs.srv import ProfileCommand
from printer3d_msgs.srv import GcodeCommand
from printer3d_msgs.srv import ImageCommand
import cv2 



class ImageCaptureNode(Node):
        def __init__(self,cameraNumber = 0):
                super().__init__('printer_image_capture')
                self.cameraNumber = cameraNumber
                self.imageService = self.create_service(ImageCommand, 'ask_capture_image', self.takePicture)

        def __del__(self):
                self.webcam.release()
                
        def takePicture(self, request, response):
                webcam = cv2.VideoCapture("/dev/video"+str(self.cameraNumber)) # Be carefull with the number after video
                check, frame = webcam.read()
                webcam.release()
                cv2.imwrite(filename=request.filename, img=frame)
                self.get_logger().info('image acquired : '+request.filename)
                response.confirmation = True
                return response


if __name__ == '__main__':
    rclpy.init()
    image_capture_node = ImageCaptureNode(2)
    rclpy.spin(image_capture_node)
    rclpy.shutdown()