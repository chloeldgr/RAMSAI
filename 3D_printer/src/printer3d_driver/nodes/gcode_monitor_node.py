#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import time
import serial
import numpy as np

from printer3d_msgs.srv import ProfileCommand
from printer3d_msgs.srv import GcodeCommand
from printer3d_msgs.srv import ImageCommand

def openSerialPort(port, baud):
        #open the serial port
        print('Opening Serial Port...')
        try:
                s = serial.Serial(port, baud)
                if (s.isOpen() == True):
                        print(port + " has opened successfully.")
                        return s
                else:
                        print(port + " has failed to open.")
                        exit()
        except:
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

                #self.get_logger().info('Gcode Received with a length of : '+str(len(gcode)))

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
                                #self.get_logger().info('Progression : '+str(feedback))

                        # Gcode Sending

                        l = removeComment(line)
                        l = l.strip()
                        self.i += 1
                        if (l.isspace() == False and len(l) > 0):
                                if ' E' in l:
                                        self.lastExtrusionSended = float(l.split('E')[1])
                                        #self.get_logger().info('extrusion changed : '+str(feedback))
                                self.serialPort.write((l + "\n").encode('ascii'))
                                grbl_out = self.serialPort.readline() # wait for response from printer
                                grbl_out = grbl_out.strip().decode('ascii')
                                while  'ok' not in grbl_out:
                                        grbl_out = self.serialPort.readline() # wait for response from printer
                                        grbl_out = grbl_out.strip().decode('ascii')


                self.serialPort.write(b"M400\n")
                grbl_out = self.serialPort.readline() # wait for response from printer
                grbl_out = grbl_out.strip().decode('ascii')
                while  'ok' not in grbl_out:
                        grbl_out = self.serialPort.readline() # wait for response from printer
                        grbl_out = grbl_out.strip().decode('ascii')


                self.serialPort.write(b"M400\n")
                grbl_out = self.serialPort.readline() # wait for response from printer
                grbl_out = grbl_out.strip().decode('ascii')
                while  'ok' not in grbl_out:
                        grbl_out = self.serialPort.readline() # wait for response from printer
                        grbl_out = grbl_out.strip().decode('ascii')


                self.get_logger().info('gcode sending completely finished')
                response.validation = True

                return response


if __name__ == '__main__':
    rclpy.init()
    gcode_monitor_node = GcodeMonitorNode()
    rclpy.spin(gcode_monitor_node)
    rclpy.shutdown()
