#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ros2_3d_printer_messages.srv import ProfileCommand
from ros2_3d_printer_messages.srv import GcodeCommand
from ros2_3d_printer_messages.srv import ImageCommand
import cv2 



class ImageCaptureNode(Node):
        def __init__(self,cameraNumber = 0):
                super().__init__('printer_image_capture')
                self.webcam = cv2.VideoCapture("/dev/video"+str(cameraNumber)) # Be carefull with the number after video
                self.imageService = self.create_service(ImageCommand, 'ask_capture_image', self.takePicture)

        def __del__(self):
                self.webcam.release()
                
        def takePicture(self,request, response):
                check, frame = self.webcam.read()
                cv2.imwrite(filename='/home/gulltor/Ramsai_Robotics/'+request.filename+'.jpg', img=frame)
                self.get_logger().info('image acquired : /home/gulltor/Ramsai_Robotics/'+request.filename+'.jpg')
                response.confirmation = True
                return response


if __name__ == '__main__':
    rclpy.init()
    image_capture_node = ImageCaptureNode(0)
    rclpy.spin(image_capture_node)
    rclpy.shutdown()