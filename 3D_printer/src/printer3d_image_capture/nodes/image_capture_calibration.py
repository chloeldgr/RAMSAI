#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from printer3d_msgs.srv import ProfileCommand
from printer3d_msgs.srv import GcodeCommand
from printer3d_msgs.srv import ImageCommand
import cv2

YPOSPHOTO = 200

class ImageCaptureNode(Node):
        def __init__(self,cameraNumber = 0):
                super().__init__('printer_image_capture')
                self.cameraNumber = cameraNumber

                self.client_printer_driver = self.create_client(GcodeCommand, 'send_gcode')
                while not self.client_printer_driver.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info('profile measure service not available, waiting again...')
                self.req_printer_driver = GcodeCommand.Request()

        def __del__(self):
                self.webcam.release()

        def showVideo(self):
                self.webcam = cv2.VideoCapture("/dev/video"+str(self.cameraNumber)) # Be carefull with the number after video
                while(True):
                        check, frame = self.webcam.read()
                        cv2.imshow('frame', frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                                break
                self.webcam.release()
                cv2.destroyAllWindows()
                return 0

        def sendGcodeSendingRequest(self,gcode):

                """ Request the sending of a certain gcode list tanken as input through the 'send_gcode' service """
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
    gcode = ['G28\n','G1 Y'+str(YPOSPHOTO)+' F2400\n']
    image_capture_node.sendGcodeSendingRequest(gcode)
    image_capture_node.showVideo()
    rclpy.shutdown()
