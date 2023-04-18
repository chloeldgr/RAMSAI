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
    // publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("gocatorPTCloud_topic", 10);
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
