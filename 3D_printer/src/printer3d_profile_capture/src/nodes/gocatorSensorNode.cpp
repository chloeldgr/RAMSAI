// Copyright 2023 ICube Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "std_msgs/msg/empty.hpp"
#include "printer3d_gocator_msgs/srv/gocator_pt_cloud.hpp"
#include "../driver/gocatorSensor.h"


using namespace std::chrono_literals;

class GocatorSensorNode : public rclcpp::Node
{
public:
  GocatorSensorNode()
  : Node("gocatorSensorNode")
  {
    gsensor_ = new GocatorSensor::Device(SENSOR_IP);
    capture_params_.exposure_time_ = 110;
    capture_params_.spacing_interval_ = 0.1;
    gsensor_->configure(capture_params_);

    service_ = this->create_service<printer3d_gocator_msgs::srv::GocatorPTCloud>(
      "gocator_get_profile",
      std::bind(
        &GocatorSensorNode::profile_snapshot, this, std::placeholders::_1,
        std::placeholders::_2));
    // publisher_ = this->create_publisher<sensor_msgs:: ...
    // msg::PointCloud2>("gocatorPTCloud_topic", 10);
  }

private:
  void profile_snapshot(
    std::shared_ptr<printer3d_gocator_msgs::srv::GocatorPTCloud::Request> request,
    std::shared_ptr<printer3d_gocator_msgs::srv::GocatorPTCloud::Response> response)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    RCLCPP_INFO(this->get_logger(), "Processing service request!");
    gsensor_->start();
    gsensor_->sendTrigger();
    if (gsensor_->getProfile(cloud) == 1) {
      auto pcloud = sensor_msgs::msg::PointCloud2();
      pcl::toROSMsg(cloud, pcloud);
      pcloud.header.frame_id = "gocator_sensor";
      pcloud.header.stamp = this->get_clock()->now();

      // publisher_->publish(pcloud);

      response->pcloud = pcloud;
    }
  }

  rclcpp::Service<printer3d_gocator_msgs::srv::GocatorPTCloud>::SharedPtr service_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  GocatorSensor::Device * gsensor_;
  GocatorSensor::CaptureParams capture_params_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GocatorSensorNode>());
  rclcpp::shutdown();
  return 0;
}
