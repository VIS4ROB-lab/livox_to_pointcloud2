#include "livox_to_pointcloud2.hpp"

using namespace std::chrono_literals;

LivoxToPointCloud2::LivoxToPointCloud2() : Node("livox_to_pointcloud2") {
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/converted_pointcloud2", 10);
  subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox_pointcloud", 10,
      std::bind(&LivoxToPointCloud2::callback, this, std::placeholders::_1));
}

void LivoxToPointCloud2::callback(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
  sensor_msgs::msg::PointCloud2 pc2_msg;
  pcl::PointCloud<livox_ros::LivoxPointXYZRTLT> output;

  output.resize(msg->point_num);

  int i = 0;
  for (const auto& point : msg->points) {
    output[i].x = point.x;
    output[i].y = point.y;
    output[i].z = point.z;
    output[i].reflectivity = point.reflectivity;
    output[i].tag = point.tag;
    output[i].line = point.line;
    output[i].offset_time = point.offset_time;
    i++;
  }

  pcl::toROSMsg(output, pc2_msg);
  pc2_msg.header.stamp = msg->header.stamp;
  pc2_msg.header.frame_id = msg->header.frame_id;

  publisher_->publish(pc2_msg);
}
