#ifndef LIVOX_TO_POINTCLOUD2_HPP
#define LIVOX_TO_POINTCLOUD2_HPP

#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <pcl/common/impl/io.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/impl/pcd_io.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"

namespace livox_ros {
struct EIGEN_ALIGN16 LivoxPointXYZRTLT {
  float x;              /**< X axis, Unit:m */
  float y;              /**< Y axis, Unit:m */
  float z;              /**< Z axis, Unit:m */
  uint8_t reflectivity; /**< Reflectivity   */
  uint8_t tag;          /**< Livox point tag   */
  uint8_t line;         /**< Laser line id     */
  uint32_t offset_time; /**< Time offset, Unit:ns */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace livox_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(
    livox_ros::LivoxPointXYZRTLT,
    (float, x, x)(float, y, y)(float, z, z)(uint8_t, reflectivity,
                                            reflectivity)(uint8_t, tag, tag)(
        uint8_t, line, line)(uint32_t, offset_time, offset_time))

class LivoxToPointCloud2 : public rclcpp::Node {
 public:
  LivoxToPointCloud2();

 private:
  void callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

#endif  // LIVOX_TO_POINTCLOUD2_HPP
