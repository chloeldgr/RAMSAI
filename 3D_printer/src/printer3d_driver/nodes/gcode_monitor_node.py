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
import time
import serial
from printer3d_msgs.srv import GcodeCommand


def openSerialPort(port, baud):
    # open the serial port
    print('Opening Serial Port...')
    try:
        s = serial.Serial(port, baud)
        if (s.isOpen() is True):
            print(port + " has opened successfully.")
            return s
        else:
            print(port + " has failed to open.")
            exit()
    except Exception:
        print("An error occurred while opening " + port)
        exit()


def removeComment(string):
    if (string.find(';') == -1):
        return string
    else:
        return string[:string.index(';')]


class GcodeMonitorNode(Node):
    def __init__(self):
        super().__init__('printer_gcode_monitor')
        self.serialPort = openSerialPort('/dev/ttyACM0', 250000)
        self.serialPort.write(b"\n\n")
        time.sleep(2)
        self.gcodeService = self.create_service(GcodeCommand, 'send_gcode', self.execute_gcode_sending)

    def execute_gcode_sending(self, request, response):
        gcode = request.gcode_strings
        self.get_logger().info('Executing Gcode')
        feedback = 0.0
        self.serialPort.flushInput()
        self.i = 0
        self.total = len(gcode)
        self.increment = 0.0

        for line in gcode:
            # feedback publication
            feedback = (self.i/self.total)*100
            if feedback - self.increment > 1:
                self.increment = feedback

            # Gcode Sending

            line = removeComment(line)
            line = line.strip()
            self.i += 1
            if (line.isspace() is False and len(line) > 0):
                if ' E' in line:
                    self.lastExtrusionSended = float(line.split('E')[1])
                self.serialPort.write((line + "\n").encode('ascii'))
                grbl_out = self.serialPort.readline()  # wait for response from printer
                grbl_out = grbl_out.strip().decode('ascii')
                while 'ok' not in grbl_out:
                    grbl_out = self.serialPort.readline()  # wait for response from printer
                    grbl_out = grbl_out.strip().decode('ascii')

        self.serialPort.write(b"M400\n")
        grbl_out = self.serialPort.readline()  # wait for response from printer
        grbl_out = grbl_out.strip().decode('ascii')
        while 'ok' not in grbl_out:
            grbl_out = self.serialPort.readline()  # wait for response from printer
            grbl_out = grbl_out.strip().decode('ascii')

        self.serialPort.write(b"M400\n")
        grbl_out = self.serialPort.readline()  # wait for response from printer
        grbl_out = grbl_out.strip().decode('ascii')
        while 'ok' not in grbl_out:
            grbl_out = self.serialPort.readline()  # wait for response from printer
            grbl_out = grbl_out.strip().decode('ascii')

        self.get_logger().info('gcode sending completely finished')
        response.validation = True

        return response


if __name__ == '__main__':
    rclpy.init()
    gcode_monitor_node = GcodeMonitorNode()
    rclpy.spin(gcode_monitor_node)
    rclpy.shutdown()
