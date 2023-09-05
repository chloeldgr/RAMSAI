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
from printer3d_gocator_msgs.srv import GocatorPTCloud
from utility import getBordersSimpleGcode, getBordersGcode, work_on_gcode_file, follow_gcode_coordinates
import printer3d_constant
import point_cloud2 as pc2
import os
import numpy as np


class PrinterControlNode(Node):
    def __init__(self):
        super().__init__('printer_control')

        """Initialisation of the printing process workspace"""
        self.declare_parameter('history_file_dir', './printing_history')
        historyFileDir = self.get_parameter('history_file_dir').get_parameter_value().string_value
        self.historyFileDir = historyFileDir
        self.get_logger().info(historyFileDir)
        self.declare_parameter('gcode_file_dir', './piece.gcode')
        self.gcodeFileDir = self.get_parameter('gcode_file_dir').get_parameter_value().string_value
        self.get_logger().info(self.gcodeFileDir)

        try:
            os.mkdir(historyFileDir)
        except Exception:
            pass

        try:
            os.mkdir(historyFileDir+'/gcode')
        except Exception:
            pass

        try:
            os.mkdir(historyFileDir+'/scan')
        except Exception:
            pass

        try:
            os.mkdir(historyFileDir+'/photos')
        except Exception:
            pass

        # access to the printer driver service

        self.client_printer_driver = self.create_client(GcodeCommand, 'send_gcode')
        while not self.client_printer_driver.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('profile measure service not available, waiting again...')
        self.req_printer_driver = GcodeCommand.Request()

        # access to the profile measure service

        self.client_profile_measure = self.create_client(GocatorPTCloud, 'gocator_get_profile')
        while not self.client_profile_measure.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('profile measure service not available, waiting again...')
        self.req_profile_measure = GocatorPTCloud.Request()

        self.imageNumber = 0
        self.scanNumber = 0

        self.last_x_value = 0
        self.last_y_value = 0
        self.last_z_value = 0
        self.last_e_value = 0

    def loadGcode(self):
        fileFullGcode = open(self.gcodeFileDir)
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

    def scanCurrentLayer(self, printedLayer):
        (layerXMin, layerYMin, layerZMin, layerXMax, layerYMax, layerZMax) = getBordersSimpleGcode(printedLayer)

        gcodeScan = [[]]
        margin = 25

        Yinit = layerYMin - printer3d_constant.GAP
        Yfinal = layerYMax - printer3d_constant.GAP
        step = 0.3

        gcodeScan[0].append('G1 Z' + str(layerZMax+1)+'\n')
        gcodeScan[0].append('G28 X0 Y0\n')
        gcodeScan[0].append('G1 Y' + str(Yinit-(margin/2)) + ' F1200\n')
        length = int((Yfinal-Yinit+margin)/step)

        self.get_logger().info('scanning begin :')
        self.get_logger().info('Yinit :' + str(Yinit))
        self.get_logger().info('Yfinal :' + str(Yfinal))
        self.get_logger().info('length :' + str(length))

        for i in range(1, length):
            gcodeScan.append([])
            gcodeScan[-1].append('G1 Y' + str(Yinit-(margin/2)+(i*step))+'\n')
        surface = []
        for movements in gcodeScan:
            self.sendGcodeSendingRequest(movements)
            profileLine = self.sendProfileMeasureRequest()
            surface.append(profileLine)
        np.save(self.historyFileDir+'/scan/layer_scan_'+str(self.scanNumber)+'.npy', np.array(surface))
        self.scanNumber += 1
        return surface

    def printCurrentLayer(self):
        return 0

    def sendProfileMeasureRequest(self):
        profile_line = []
        self.future_profile_measure = self.client_profile_measure.call_async(self.req_profile_measure)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future_profile_measure.done():
                try:
                    self.response = self.future_profile_measure.result()
                except Exception as e:
                    self.get_logger().info(f'Service call failed {e!r}')
                else:
                    pcloud = self.response.pcloud
                    self.flag_datas_used = True

                    gen = pc2.read_points(pcloud, skip_nans=True)
                    profile = list(gen)

                    for points in profile:
                        profile_line.append([[points[0], 0, points[2]]])
                break
        return profile_line

    def printLastPositions(self):
        self.get_logger().info("Last Position : ("+str(self.last_x_value)+", "+str(self.last_y_value)+", "+str(self.last_z_value)+", "+str(self.last_e_value)+")\n")
        return 0

    def sendGcodeSendingRequest(self, gcode):
        assert self.verifyGcodeBeforeSending(gcode) is True

        (changed_x,changed_y,changed_z,changed_e) = follow_gcode_coordinates(gcode)
        if changed_x != None:
            self.last_x_value = changed_x
        if changed_y != None:
            self.last_y_value = changed_y
        if changed_z != None:
            self.last_z_value = changed_z
        if changed_e != None:
            self.last_e_value = changed_e

        self.req_printer_driver.gcode_strings = gcode
        self.future_printer_driver = self.client_printer_driver.call_async(self.req_printer_driver)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future_printer_driver.done():
                break

if __name__ == '__main__':
    carriageReturn = ['G28\n']
    rclpy.init()
    printer_control_node = PrinterControlNode()
    printer_control_node.sendGcodeSendingRequest(carriageReturn)
    gcode = printer_control_node.loadGcode()
    for i in range(0, len(gcode)-1):
        printer_control_node.get_logger().info('lancement de la couche '+str(i+1)+' sur '+str(len(gcode)-1))
        printer_control_node.sendGcodeSendingRequest(gcode[i])
        printer_control_node.printLastPositions()
        printer_control_node.get_logger().info('fin de la couche '+str(i+1)+' sur '+str(len(gcode)))
        printer_control_node.scanCurrentLayer(gcode[i])
        
    rclpy.shutdown()
