#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from printer3d_msgs.srv import ProfileCommand
from printer3d_msgs.srv import GcodeCommand
from printer3d_msgs.srv import ImageCommand
from printer3d_gocator_msgs.srv import GocatorPTCloud
from utility import *
import printer3d_constant
from sensor_msgs.msg import PointCloud2
import point_cloud2 as pc2
import os
import numpy as np



class PrinterControlNode(Node):
        def __init__(self, historyFilename):
                super().__init__('printer_control')

                """Initialisation of the printing process workspace"""
                self.historyFilename = historyFilename
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

                self.imageNumber = 0
                self.scanNumber = 0


        def loadGcode(self,gcodeFilename):
                fileFullGcode = open(gcodeFilename)
                rawGode = fileFullGcode.readlines()
                fileFullGcode.close()

                (self.xMin,self.yMin,self.zMin,self.xMax,self.yMax,self.zMax) = getBordersGcode(rawGode)
                self.get_logger().info('xMin : '+str(self.xMin))
                self.get_logger().info('xMax : '+str(self.xMax))
                self.get_logger().info('yMin : '+str(self.yMin))
                self.get_logger().info('yMax : '+str(self.yMax))
                self.get_logger().info('zMin : '+str(self.zMin))
                self.get_logger().info('zMax : '+str(self.zMax))

                assert self.xMin >= printer3d_constant.XMIN
                assert self.xMax <= printer3d_constant.XMAX

                assert self.yMin >= printer3d_constant.YMIN
                assert self.yMax <= printer3d_constant.YMAX

                assert self.zMin >= printer3d_constant.ZMIN
                assert self.zMax <= printer3d_constant.ZMAX
                self.gcodeLayers = work_on_gcode_file(rawGode)

                return self.gcodeLayers

        def verifyGcodeBeforeSending(self,gcodeToVerify):

                (self.xMin,self.yMin,self.zMin,self.xMax,self.yMax,self.zMax) = getBordersSimpleGcode(gcodeToVerify)
                testXMin = self.xMin >= printer3d_constant.XMIN
                testXMax = self.xMax <= printer3d_constant.XMAX

                testYMin = self.yMin >= printer3d_constant.YMIN
                testYMax = self.yMax <= printer3d_constant.YMAX

                testZMin = self.zMin >= printer3d_constant.ZMIN
                testZMax = self.zMax <= printer3d_constant.ZMAX

                if testXMin == False or testXMax == False or testYMin == False or testYMax == False or testZMin == False or testZMax == False:
                        return False
                else:
                        return True

        def scanCurrentLayer(self,printedLayer):
                (layerXMin,layerYMin,layerZMin,layerXMax,layerYMax,layerZMax) = getBordersSimpleGcode(printedLayer)

                gcodeScan = [[]]
                margin = 25

                Yinit = layerYMin - printer3d_constant.GAP
                Yfinal = layerYMax - printer3d_constant.GAP
                step = 0.3

                gcodeScan[0].append('G1 Z'+str(layerZMax+1)+'\n')
                gcodeScan[0].append('G28 X0 Y0\n')
                gcodeScan[0].append('G1 Y'+str(Yinit-(margin/2))+' F1200\n')
                length = int((Yfinal-Yinit+margin)/step)

                self.get_logger().info('scanning begin :')
                self.get_logger().info('Yinit :' + str(Yinit))
                self.get_logger().info('Yfinal :'+ str(Yfinal))
                self.get_logger().info('length :'+ str(length))

                for i in range(1,length):
                        gcodeScan.append([])
                        gcodeScan[-1].append('G1 Y'+str(Yinit-(margin/2)+(i*step))+'\n')
                surface = []
                for movements in gcodeScan:
                        self.sendGcodeSendingRequest(movements)
                        profileLine = self.sendProfileMeasureRequest()
                        surface.append(profileLine)
                np.save('/home/gulltor/Ramsai_Robotics/history/'+self.historyFilename+'/scan/layer_scan_'+str(self.scanNumber)+'.npy',np.array(surface))
                self.scanNumber += 1
                return surface

        def printCurrentLayer(self):
                return 0

        def takeCurrentLayerPhoto(self):
                gcodeMoveToPos = ['G28 X0 Y0\n','G1 Y'+str(printer3d_constant.YPOSPHOTO)+' F2400\n']
                self.sendGcodeSendingRequest(gcodeMoveToPos)
                self.sendImageCaptureRequest('/home/gulltor/Ramsai_Robotics/history/'+self.historyFilename+'/photos/layer_photo_'+str(self.imageNumber)+'.jpg')
                self.imageNumber += 1
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
                                        self.get_logger().info('Service call failed {!r}'.format(e))
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

                assert self.verifyGcodeBeforeSending(gcode) == True

                self.req_printer_driver.gcode_strings = gcode
                self.future_printer_driver = self.client_printer_driver.call_async(self.req_printer_driver)
                while rclpy.ok():
                        rclpy.spin_once(self)
                        if self.future_printer_driver.done():
                                self.get_logger().info('gcode sended')
                                break

if __name__ == '__main__':
        carriageReturn = ['G28\n']
        rclpy.init()
        printer_control_node = PrinterControlNode('impression_base_2')
        printer_control_node.sendGcodeSendingRequest(carriageReturn)
        gcode = printer_control_node.loadGcode('/home/gulltor/Ramsai_Robotics/Gcodes/piece_test_base.gcode')
        for i in range(90,len(gcode)):
                # printer_control_node.sendGcodeSendingRequest(gcode[i])
                printer_control_node.takeCurrentLayerPhoto()
                printer_control_node.scanCurrentLayer(gcode[i])
        rclpy.shutdown()
