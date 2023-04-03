import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from 3D_printer_messages.srv import profileCommand
from 3D_printer_messages.srv import gcodeCommand
from 3D_printer_messages.srv import imageCommand
import cv2 



class ImageCaptureNode(Node):
        def __init__(self,cameraNumber = 0):
                super().__init__('3D_printer_image_capture')
                self.webcam = cv2.VideoCapture("/dev/video"+str(cameraNumber)) # Be carefull with the number after video
                self.imageService = self.create_service(imageCommand, 'ask_capture_image', self.takePicture)

               
        def __del__(self):
                webcam.release()
                
        def takePicture(self,request, response):
                check, frame = webcam.read()
                cv2.imwrite(filename='/home/gulltor/Ramsai_Robotics/images/'+request.filename+'.jpg', img=frame)

                response.confirmation = True
                return response


if __name__ == '__main__':
    rclpy.init()
    image_capture_node = ImageCaptureNode(0)
    rclpy.spin(image_capture_node)
    rclpy.shutdown()