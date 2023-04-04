#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from printer3d_msgs.srv import ProfileCommand
from printer3d_msgs.srv import GcodeCommand
from printer3d_msgs.srv import ImageCommand
from printer3d_gocator_msgs.srv import GocatorPTCloud

from utility import *

from sensor_msgs.msg import PointCloud2 
import point_cloud2 as pc2
import os


class PrinterControlNode(Node):
        def __init__(self, historyFilename):
                super().__init__('printer_control') 


                """Initialisation of the printing process workspace"""

                try:
                        os.mkdir('/home/gulltor/Ramsai_Robotics/history/'+historyFilename)
                except:
                        pass

                try:
                        os.mkdir('/home/gulltor/Ramsai_Robotics/history/'+historyFilename+'/gcode')
                except:
                        pass

                try:
                        os.mkdir('/home/gulltor/Ramsai_Robotics/history/'+historyFilename+'/scan')
                except:
                        pass

                try:
                        os.mkdir('/home/gulltor/Ramsai_Robotics/history/'+historyFilename+'/photos')
                except:
                        pass

                # access to the image capture service

                self.client_image_capture = self.create_client(ImageCommand, 'ask_capture_image')
                while not self.client_image_capture.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info('image capture service not available, waiting again...')
                self.req_image_capture = ImageCommand.Request()

                # access to the profile measure service

                self.client_profile_measure = self.create_client(GocatorPTCloud, 'gocator_get_profile')
                while not self.client_profile_measure.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info('profile measure service not available, waiting again...')
                self.req_profile_measure = GocatorPTCloud.Request()

                # access to the printer driver service

                self.client_printer_driver = self.create_client(GcodeCommand, 'send_gcode')
                while not self.client_printer_driver.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info('profile measure service not available, waiting again...')
                self.req_printer_driver = GcodeCommand.Request()


        def loadGcode(self,gcodeFilename):
                fileFullGcode = open(file,'r')
                rawGode = fichier.readlines()
                fileFullGcode.close()

                (self.xMin,self.yMin,self.zMin,self.xMax,self.yMax,self.zMax) = getBordersGcode(rawGode)
                self.gcodeLayers = work_on_gcode_file(gco)

        def scanCurrentLayer(self):
                return 0

        def printCurrentLayer(self):
                return 0

        def takeCurrentLayerPhoto(self):
                return 0

        def sendImageCaptureRequest(self,filename):

                """ Request an image capture using a webcam through the 'ask_image_capture' service """

                self.req_image_capture.filename = filename
                self.future_image_capture = self.client_image_capture.call_async(self.req_image_capture)
                while rclpy.ok():
                        rclpy.spin_once(self)
                        if self.future_image_capture.done():
                                self.get_logger().info('capture finished')
                                break

        def sendProfileMeasureRequest(self):

                """ Request a profile measure using a Gocator 2140 profile sensor through the 'gocator_get_profile' service """

                profile_line = []
                self.future_profile_measure = self.client_profile_measure.call_async(self.req_profile_measure)
                while rclpy.ok():
                        rclpy.spin_once(self)
                        if self.future_profile_measure.done():
                                try:
                                        self.response = self.future_profile_measure.result()
                                except Exception as e:
                                        self.get_logger().info('Service call failed %r' % (e,))
                                else:
                                        pcloud = self.response.pcloud
                                        self.flag_datas_used = True
                                        
                                        gen = pc2.read_points(pcloud, skip_nans=True)
                                        profile = list(gen)

                                        for points in profile:
                                                profile_line.append([[points[0],0,points[2]]])
                                break
                self.get_logger().info('scan finished')
                return profile_line  

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
        gcodeExample = ['G28\n','G1 F3000 X10 Y10 Z10\n','G28\n']

        rclpy.init()
        printer_control_node = PrinterControlNode('impression_test')
        for i in range(0,10):
                printer_control_node.sendImageCaptureRequest('impressions/test/image_test_'+str(i))
        printer_control_node.sendGcodeSendingRequest(gcodeExample)
        profile = printer_control_node.sendProfileMeasureRequest()

        rclpy.shutdown()