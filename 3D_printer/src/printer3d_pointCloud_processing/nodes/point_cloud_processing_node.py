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
import numpy as np
import pyvista as pv
import matplotlib.pyplot as plt
import matplotlib
import random

""" parametres du noeuds ROS, à mettre dans un fichier de config a la fin"""
basic_limit_for_reflexions = 12


coords = []

def onclick(event):
    ix, iy = event.xdata, event.ydata
    print ('x = ',ix,', y = ',iy)
    global coords
    coords.append((ix, iy))
    if len(coords) == 2:
        print('ok')
        plt.close()
    return coords

class PointCloudProcessingNode(Node):
    def __init__(self):
        """PointCloudProcessingNode class constructor."""
        super().__init__('point_cloud_processing_node')

    def load_scan(self,fileName):
        data = np.squeeze(np.load(fileName))
        xDim = data.shape[1]
        yDim = data.shape[0]
        zDim = data.shape[2]
        for i in range(0,yDim):
            for j in range(0,xDim):
                data[i,j,1] = i*0.3
        return data

    def decimate_point_cloud(self,inputPointCloud,decimationRate):
        shapes = inputPointCloud.shape
        interPointCloud = self.flatten_point_cloud(inputPointCloud)
        indexList = [i for i in range(0,interPointCloud.shape[0])]
        deletedElementsIndex = random.sample(indexList,int(len(indexList)*decimationRate))
        outputPointCloud = np.delete(interPointCloud, deletedElementsIndex,axis=0)
        return outputPointCloud

    def flatten_point_cloud(self,inputPointCloud):
        if len(inputPointCloud.shape) == 2:
            outputPointCloud = inputPointCloud
        else:
            outputPointCloud = np.reshape(inputPointCloud,(inputPointCloud.shape[0]*inputPointCloud.shape[1],inputPointCloud.shape[2]))
        return outputPointCloud

    def filter_point_cloud(self,inputPointCloud):
        treatedPointCloud = self.remove_inf_value(inputPointCloud)
        shapes = treatedPointCloud.shape
        savedPoints = []
        for i in range(0,shapes[0]):
            if treatedPointCloud[i,0]>self.filter_coordinates[0][0] and treatedPointCloud[i,0]<self.filter_coordinates[1][0] and treatedPointCloud[i,1]>self.filter_coordinates[0][1] and treatedPointCloud[i,1]<self.filter_coordinates[1][1]:
                savedPoints.append(treatedPointCloud[i,:])
        savedPoints = np.array(savedPoints)
        self.plot_point_cloud(savedPoints)
        return 0

    def remove_inf_value(self,inputPointCloud):
        inputPointCloud = self.flatten_point_cloud(inputPointCloud)
        shapes = inputPointCloud.shape
        outputPointCloud = []
        for i in range(0,shapes[0]):
            if inputPointCloud[i,2]<10000 and inputPointCloud[i,2]>-10000:
                outputPointCloud.append(inputPointCloud[i,:])
        return np.array(outputPointCloud)

    def plot_point_cloud(self,inputPointCloud):
        data = self.remove_inf_value(inputPointCloud)
        data = self.decimate_point_cloud(data, 0.95)
        x = data[:,0]
        y = data[:,1]
        z = data[:,2]

        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d',aspect='auto')
        im = ax.scatter(x, y, z, c=z)
        plt.xlabel('x (mm)')
        plt.ylabel('y (mm)')
        fig.colorbar(im)
        ax.set_xlim(min([min(x), min(y), min(z)]), max([max(x), max(y), max(z)]))
        ax.set_ylim(min([min(x), min(y), min(z)]), max([max(x), max(y), max(z)]))
        ax.set_zlim(min([min(x), min(y), min(z)]), max([max(x), max(y), max(z)]))
        plt.show()
        return 0

    def ask_for_limits(self,inputPointCloud):
        data = self.remove_inf_value(inputPointCloud)
        data = self.decimate_point_cloud(data, 0.95)
        x = data[:,0]
        y = data[:,1]
        z = data[:,2]
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111,aspect='equal')
        im = ax.scatter(x, y, c=z,s=1)
        plt.xlabel('x (mm)')
        plt.ylabel('y (mm)')
        fig.colorbar(im)
        ax.set_xlim(min(x)-10, max(x)+10)
        ax.set_ylim(min(y)-10, max(y)+10)
        cid = fig.canvas.mpl_connect('button_press_event', onclick)
        plt.show()
        global coords
        self.filter_coordinates = coords
        coords = []
        return 0

    def extract_reference_base_change(self):
        return 0

if __name__ == '__main__':
    rclpy.init()
    point_cloud_processing_node = PointCloudProcessingNode()
    scan = point_cloud_processing_node.load_scan("/home/gulltor/Ramsai_Robotics/history/impression_base_avec_retraction_20_pourcents/scan/layer_scan_4.npy")
    point_cloud_processing_node.ask_for_limits(scan)
    filtered_scan = point_cloud_processing_node.filter_point_cloud(scan)
    rclpy.shutdown()
