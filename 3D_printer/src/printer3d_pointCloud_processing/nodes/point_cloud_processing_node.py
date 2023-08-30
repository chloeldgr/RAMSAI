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

matplotlib.use('GTK3agg',force=True)



class PointCloudProcessingNode(Node):
    def __init__(self):
        """PointCloudProcessingNode class constructor."""
        super().__init__('point_cloud_processing_node')

    def load_scan(self,fileName):
        data = np.squeeze(np.load(fileName))
        print(type(data))
        print(data.shape)
        data = np.squeeze(data)
        plt.figure()
        plt.plot(data[0][:][2])
        plt.show()
        print(data.shape)
        return 0

    def extract_reference_base_change(self):
        return 0



if __name__ == '__main__':
    rclpy.init()
    print(plt.get_backend())
    point_cloud_processing_node = PointCloudProcessingNode()
    scan = point_cloud_processing_node.load_scan("/home/gulltor/impression_base_avec_retraction_20_pourcents/scan/layer_scan_0.npy")
    rclpy.shutdown()
