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

GroundSegmetation ground_seg;

ros::Publisher non_ground_pub;
ros::Publisher height_marker_pub;

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
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_non_ground(new pcl::PointCloud<pcl::PointXYZI>);
  std::vector<WallSlice> height_slice;
  double slope_angle;
  ground_seg.ground_sgementation(cloud_msg,cloud_non_ground,height_slice,slope_angle);
  sensor_msgs::PointCloud2 cloud_msg2;
  pcl::toROSMsg(*cloud_non_ground, cloud_msg2);
  cloud_msg2.header.frame_id = "base_link";  // 或你的坐标系
  cloud_msg2.header.stamp = ros::Time::now();
  non_ground_pub.publish(cloud_msg2);

  visualization_msgs::MarkerArray marker_array;
  int id = 0;

  for (const auto& slice : height_slice) {
      visualization_msgs::Marker text;
      text.header.frame_id = "base_link";
      text.header.stamp = ros::Time::now();
      text.ns = "wall_heights";
      text.id = id++;
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;

      text.pose.position.x = -20;                  // 固定在墙体 x 附近
      text.pose.position.y = slice.y_center;
      text.pose.position.z = slice.z_max + 0.3;    // 提高显示位置
      text.pose.orientation.w = 1.0;

      text.scale.z = 0.4;   // 字体高度
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 0.0;
      text.color.a = 1.0;

      std::ostringstream oss;
      oss << "Y: " << std::fixed << std::setprecision(1) << slice.y_center
          << "m\nH: " << std::fixed << std::setprecision(2) << slice.height() << "m";
      text.text = oss.str();

      marker_array.markers.push_back(text);
  }

  height_marker_pub.publish(marker_array);

  WallInfo info;
  // info.wall_height = max_pt.z;
  info.back_slope_angle = slope_angle;  // 这里只是示例，实际需要计算
  info.latitude = current_lat;
  info.longitude = current_lon;

  publishToMQTT(info, cloud_non_ground);
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

  non_ground_pub = nh.advertise<sensor_msgs::PointCloud2>("non_ground_cloud", 1);
  height_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("wall_height_markers", 1);

  ros::spin();

  mqtt_client->disconnect()->wait();
  delete mqtt_client;
  return 0;
}
