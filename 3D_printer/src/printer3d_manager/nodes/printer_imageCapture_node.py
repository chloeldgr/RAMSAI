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
from printer3d_msgs.srv import ImageCommand
from utility import getBordersSimpleGcode, getBordersGcode, work_on_gcode_file
import printer3d_constant
import os
import numpy as np


class PrinterControlNode(Node):
    def __init__(self, historyFilename):
        super().__init__('printer_control')

        """Initialisation of the printing process workspace"""
        self.historyFilename = historyFilename
        try:
            os.mkdir('/home/gulltor/Ramsai_Robotics/history/'+historyFilename)
        except Exception:
            pass

        try:
            os.mkdir('/home/gulltor/Ramsai_Robotics/history/'+historyFilename+'/gcode')
        except Exception:
            pass

        try:
            os.mkdir('/home/gulltor/Ramsai_Robotics/history/'+historyFilename+'/scan')
        except Exception:
            pass

        try:
            os.mkdir('/home/gulltor/Ramsai_Robotics/history/'+historyFilename+'/photos')
        except Exception:
            pass

        # access to the image capture service

        self.client_image_capture = self.create_client(ImageCommand, 'ask_capture_image')
        while not self.client_image_capture.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('image capture service not available, waiting again...')
        self.req_image_capture = ImageCommand.Request()

        # access to the printer driver service

        self.client_printer_driver = self.create_client(GcodeCommand, 'send_gcode')
        while not self.client_printer_driver.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('profile measure service not available, waiting again...')
        self.req_printer_driver = GcodeCommand.Request()

        self.imageNumber = 0
        self.scanNumber = 0

    def loadGcode(self, gcodeFilename):
        fileFullGcode = open(gcodeFilename)
        rawGode = fileFullGcode.readlines()
        fileFullGcode.close()

        (self.xMin, self.yMin, self.zMin, self.xMax, self.yMax, self.zMax) = getBordersGcode(rawGode)
        self.get_logger().info('xMin : ' + str(self.xMin))
        self.get_logger().info('xMax : ' + str(self.xMax))
        self.get_logger().info('yMin : ' + str(self.yMin))
        self.get_logger().info('yMax : ' + str(self.yMax))
        self.get_logger().info('zMin : ' + str(self.zMin))
        self.get_logger().info('zMax : ' + str(self.zMax))

        assert self.xMin >= printer3d_constant.XMIN
        assert self.xMax <= printer3d_constant.XMAX

        assert self.yMin >= printer3d_constant.YMIN
        assert self.yMax <= printer3d_constant.YMAX

        assert self.zMin >= printer3d_constant.ZMIN
        assert self.zMax <= printer3d_constant.ZMAX
        self.gcodeLayers = work_on_gcode_file(rawGode)

        return self.gcodeLayers

    def verifyGcodeBeforeSending(self, gcodeToVerify):
        (self.xMin, self.yMin, self.zMin, self.xMax, self.yMax, self.zMax) = getBordersSimpleGcode(gcodeToVerify)
        testXMin = self.xMin >= printer3d_constant.XMIN
        testXMax = self.xMax <= printer3d_constant.XMAX

        testYMin = self.yMin >= printer3d_constant.YMIN
        testYMax = self.yMax <= printer3d_constant.YMAX

        testZMin = self.zMin >= printer3d_constant.ZMIN
        testZMax = self.zMax <= printer3d_constant.ZMAX

        if testXMin is False or testXMax is False or testYMin is False or testYMax is False or testZMin is False or testZMax is False:
            return False
        else:
            return True

    def printCurrentLayer(self):
        return 0

    def takeCurrentLayerPhoto(self):
        gcodeMoveToPos = ['G28 X0 Y0\n', 'G1 Y' + str(printer3d_constant.YPOSPHOTO) + ' F2400\n']
        self.sendGcodeSendingRequest(gcodeMoveToPos)
        self.sendImageCaptureRequest('/home/gulltor/Ramsai_Robotics/history/'+self.historyFilename+'/photos/layer_photo_'+str(self.imageNumber)+'.jpg')
        self.imageNumber += 1
        return 0

    def sendImageCaptureRequest(self, filename):
        self.req_image_capture.filename = filename
        self.future_image_capture = self.client_image_capture.call_async(self.req_image_capture)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future_image_capture.done():
                self.get_logger().info('capture finished')
                break

    def sendGcodeSendingRequest(self, gcode):
        assert self.verifyGcodeBeforeSending(gcode) is True

        self.req_printer_driver.gcode_strings = gcode
        self.future_printer_driver = self.client_printer_driver.call_async(self.req_printer_driver)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future_printer_driver.done():
                break

if __name__ == '__main__':
    carriageReturn = ['G28\n']
    rclpy.init()
    printer_control_node = PrinterControlNode('impression_base_50_pourcents')
    printer_control_node.sendGcodeSendingRequest(carriageReturn)
    gcode = printer_control_node.loadGcode('/home/gulltor/Ramsai_Robotics/Gcodes/piece_test_base.gcode')
    for i in range(0, len(gcode)-1):
        printer_control_node.get_logger.info('lancement de la couche '+str(i+1)+' sur '+str(len(gcode)-1))
        printer_control_node.sendGcodeSendingRequest(gcode[i])
        printer_control_node.get_logger.info('fin de la couche '+str(i+1)+' sur '+str(len(gcode)))
        printer_control_node.takeCurrentLayerPhoto()
        
    rclpy.shutdown()
