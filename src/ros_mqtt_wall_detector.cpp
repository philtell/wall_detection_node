// ros_mqtt_wall_detector.cpp

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <gps_common/GPSFix.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include <sstream>
#include "cloud_process.h"

using json = nlohmann::json;

struct WallInfo {
  double wall_height;
  double back_slope_angle;
  double latitude;
  double longitude;
};

double current_lat = 0.0;
double current_lon = 0.0;
mqtt::async_client* mqtt_client;

void publishToMQTT(const WallInfo& info, const pcl::PointCloud<pcl::PointXYZI>::Ptr& wall_cloud) {
  // 发布结构化信息
  json j;
  j["wall_height"] = info.wall_height;
  j["slope_angle"] = info.back_slope_angle;
  j["lat"] = info.latitude;
  j["lon"] = info.longitude;
  std::cout<<"I have send height:"<< j["wall_height"]<<std::endl;
  mqtt::message_ptr pubmsg = mqtt::make_message("wall/info", j.dump());
  pubmsg->set_qos(1);
  mqtt_client->publish(pubmsg);

  // 发布点云信息（简单序列化为JSON数组）
  json cloud_json = json::array();
  for (const auto& pt : wall_cloud->points) {
    cloud_json.push_back({ pt.x, pt.y, pt.z });
      std::cout<<"x:"<< pt.x<<" Y:"<<pt.y<<" z:"<<pt.z<<std::endl;
  }
  mqtt::message_ptr cloud_msg = mqtt::make_message("wall/cloud", cloud_json.dump());
  cloud_msg->set_qos(1);
  mqtt_client->publish(cloud_msg);
}

void gpsCallback(const gps_common::GPSFixConstPtr& msg) {
  current_lat = msg->latitude;
  current_lon = msg->longitude;
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // 简单高度过滤模拟挡墙提取
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(1.5, 5.0);  // 假设挡墙高度在这个范围
  pcl::PointCloud<pcl::PointXYZI>::Ptr wall_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pass.filter(*wall_cloud);

  // 计算最高点作为挡墙高度
  pcl::PointXYZI min_pt, max_pt;
  pcl::getMinMax3D(*wall_cloud, min_pt, max_pt);

  WallInfo info;
  info.wall_height = max_pt.z;
  info.back_slope_angle = 15.0;  // 这里只是示例，实际需要计算
  info.latitude = current_lat;
  info.longitude = current_lon;

  publishToMQTT(info, wall_cloud);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "wall_detector_node");
  ros::NodeHandle nh;

  // 初始化 MQTT 客户端
  mqtt_client = new mqtt::async_client("tcp://localhost:1883", "ros_wall_detector");
  mqtt::connect_options connOpts;
  mqtt_client->connect(connOpts)->wait();

  // ROS 订阅
  ros::Subscriber cloud_sub = nh.subscribe("lidar_back/raw_cloud", 10, cloudCallback);
  ros::Subscriber rtk_sub = nh.subscribe("/gps/fix", 10, gpsCallback);

  ros::spin();

  mqtt_client->disconnect()->wait();
  delete mqtt_client;
  return 0;
}
