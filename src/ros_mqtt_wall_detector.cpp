// ros_mqtt_wall_detector.cpp

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <gps_common/GPSFix.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Imu.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <nlohmann/json.hpp>
#include <sstream>
#include "cloud_process.h"
#include <chrono>
#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <chrono>


int sock_fd;
struct sockaddr_in server_addr;

using json = nlohmann::json;


struct WallPoint {
  double x;
  double y;
  double z;
};

struct WallInfo {
  std::vector<WallPoint> wall_points;  // 表示多个墙体点
  double back_slope_angle;
  double latitude;
  double longitude;
};


double current_lat = 0.0;
double current_lon = 0.0;

GroundSegmetation ground_seg;

ros::Publisher non_ground_pub;
ros::Publisher height_marker_pub;
ros::Publisher marker_pub;

Eigen::Quaternionf imu_orientation = Eigen::Quaternionf::Identity();
std::mutex imu_mutex;



// 替代 mqtt_client = new mqtt::async_client(...) 等初始化代码
void initSocket(const std::string& ip, int port) {
  sock_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (sock_fd < 0) {
      perror("Socket creation failed");
      exit(EXIT_FAILURE);
  }

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr);

  while (true) {
      if (connect(sock_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) == 0) {
          ROS_INFO("Connected to socket server at %s:%d", ip.c_str(), port);
          break;
      } else {
          perror("Socket connection failed, retrying in 1s...");
          std::this_thread::sleep_for(std::chrono::seconds(1));
      }
  }
}


void publishToSocket(const WallInfo& info, const pcl::PointCloud<pcl::PointXYZI>::Ptr& wall_cloud) {
  json j;
  j["wall_points"] = json::array();
  for (const auto &pt : info.wall_points) {
      j["wall_points"].push_back({{"x", pt.x}, {"y", pt.y}, {"z", pt.z}});
  }
  j["slope_angle"] = info.back_slope_angle;
  j["lat"] = info.latitude;
  j["lon"] = info.longitude;

  json cloud_json = json::array();
  for (const auto& pt : wall_cloud->points) {
      cloud_json.push_back({ pt.x, pt.y, pt.z });
  }
  j["cloud"] = cloud_json;

  std::string payload = j.dump();
  int ret = send(sock_fd, payload.c_str(), payload.size(), 0);
  if (ret < 0) {
      perror("Socket send failed");
  } else {
      ROS_INFO("Wall info sent to socket, size: %ld bytes", payload.size());
  }
}


// 发布平面
void publishGroundPlaneMarker(const pcl::ModelCoefficients::Ptr& coefficients,
                              ros::Publisher& marker_pub,
                              const std::string& frame_id = "map")
{
  // 提取平面参数 ax + by + cz + d = 0
  float a = coefficients->values[0];
  float b = coefficients->values[1];
  float c = coefficients->values[2];
  float d = coefficients->values[3];

  // 创建平面 marker（以一个矩形表示）
  visualization_msgs::Marker marker;
  marker.header.frame_id = "perception";  // 或你的坐标系
  marker.header.stamp = ros::Time::now();
  marker.ns = "ground_plane";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.color.a = 0.5;
  marker.color.r = 0.2;
  marker.color.g = 0.8;
  marker.color.b = 0.2;

  // 构建矩形平面顶点（大小为10m x 10m）
  float size = 10.0;
  std::vector<Eigen::Vector3f> corners;
  corners.emplace_back(-size, 0, 0);
  corners.emplace_back(size, 0, 0);
  corners.emplace_back(size, size*2, 0);
  corners.emplace_back(-size, size*2, 0);

  // 将每个角点投影到平面上
  for (auto& pt : corners)
  {
    float t = -(a * pt.x() + b * pt.y() + c * pt.z() + d) / (a * a + b * b + c * c);
    pt = pt + t * Eigen::Vector3f(a, b, c);
  }

  // 构建两个三角形
  std::vector<int> indices = {0, 1, 2, 0, 2, 3};
  for (int idx : indices)
  {
    geometry_msgs::Point p;
    p.x = corners[idx].x();
    p.y = corners[idx].y();
    p.z = corners[idx].z();
    marker.points.push_back(p);
  }

  marker_pub.publish(marker);
}

void gpsCallback(const gps_common::GPSFixConstPtr& msg) {
  current_lat = msg->latitude;
  current_lon = msg->longitude;
  ROS_INFO("lat: %f, lon: %f", current_lat, current_lon);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    std::lock_guard<std::mutex> lock(imu_mutex);
    imu_orientation.w() = imu_msg->orientation.w;
    imu_orientation.x() = imu_msg->orientation.x;
    imu_orientation.y() = imu_msg->orientation.y;
    imu_orientation.z() = imu_msg->orientation.z;
    ROS_INFO("IMU orientation updated. w: %f, x: %f, y: %f, z: %f",
             imu_orientation.w(), imu_orientation.x(), imu_orientation.y(), imu_orientation.z());
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_non_ground(new pcl::PointCloud<pcl::PointXYZI>);
  // 记录程序开始时间
  auto start_time = std::chrono::high_resolution_clock::now();


  Eigen::Quaternionf imu_q;
  {
      std::lock_guard<std::mutex> lock(imu_mutex);
      imu_q = imu_orientation;
  }

  // 将点云从车身坐标变换到水平坐标（即去除roll和pitch）
  Eigen::Matrix3f R = imu_q.toRotationMatrix();

  // 提取 roll 和 pitch 的反旋转
  Eigen::Vector3f euler = R.eulerAngles(0, 1, 2); // roll pitch yaw
  float roll = euler[0], pitch = euler[1];

  // 构造反旋转矩阵，只矫正 roll 和 pitch
  Eigen::Matrix3f inv_rot;
  inv_rot = Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(-roll,  Eigen::Vector3f::UnitX());

  Eigen::Affine3f transform(inv_rot);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::fromROSMsg(*cloud_msg, *cloud_in);  
  pcl::transformPointCloud(*cloud_in, *cloud_out, transform);

  // 发布矫正后的点云（可选）
  sensor_msgs::PointCloud2 corrected_msg;
  pcl::toROSMsg(*cloud_out, corrected_msg);
  corrected_msg.header = cloud_msg->header;
  std::vector<WallSlice> height_slice;
  double slope_angle;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  ground_seg.ground_sgementation(cloud_msg,cloud_non_ground,height_slice,slope_angle,coefficients);
  sensor_msgs::PointCloud2 cloud_msg2;
  pcl::toROSMsg(*cloud_non_ground, cloud_msg2);
  cloud_msg2.header.frame_id = "perception";  // 或你的坐标系
  cloud_msg2.header.stamp = ros::Time::now();
  non_ground_pub.publish(cloud_msg2);
  ROS_INFO("No Ground points: %zu", cloud_non_ground->points.size());
  publishGroundPlaneMarker(coefficients, marker_pub);
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  WallInfo info;
  info.wall_points.clear();
  for (const auto& slice : height_slice) {
      WallPoint pt;
      pt.x = slice.x_center;                    // 挡墙 X 位置固定
      pt.y = slice.y_center;          // Y 中心
      pt.z = slice.height();          // Z 为高度
      info.wall_points.push_back(pt);
      visualization_msgs::Marker text;
      text.header.frame_id = "perception";
      text.header.stamp = ros::Time::now();
      text.ns = "wall_heights";
      text.id = id++;
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;

      text.pose.position.x = slice.x_center;                  // 固定在墙体 x 附近
      text.pose.position.y = slice.y_center;
      text.pose.position.z = slice.z_max + 0.3;    // 提高显示位置
      text.pose.orientation.w = 1.0;

      text.scale.z = 0.4;   // 字体高度
      text.color.r = 1.0;
      text.color.g = 0.0;
      text.color.b = 0.0;
      text.color.a = 1.0;

      std::ostringstream oss;
      oss << "H:" << std::fixed << std::setprecision(2) << slice.height() << "m";
      text.text = oss.str();

      marker_array.markers.push_back(text);
  }

  height_marker_pub.publish(marker_array);

  info.back_slope_angle = slope_angle;  // 这里只是示例，实际需要计算
  info.latitude = current_lat;
  info.longitude = current_lon;

  publishToSocket(info, cloud_non_ground);

  // publishToMQTT(info, cloud_non_ground);

  // 程序退出前，记录结束时间
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

  ROS_INFO_STREAM("Cost Time: " << duration << " ms");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "wall_detector_node");
  ros::NodeHandle nh;

  initSocket("127.0.0.1", 9000);  // 你可以替换为目标服务器的 IP 和端口


  // ROS 订阅
  ros::Subscriber cloud_sub = nh.subscribe("lidar_back/raw_cloud", 10, cloudCallback);
  ros::Subscriber rtk_sub = nh.subscribe("rtk_test", 10, gpsCallback);
  ros::Subscriber imu_sub = nh.subscribe("imu_test", 10, imuCallback);

  non_ground_pub = nh.advertise<sensor_msgs::PointCloud2>("non_ground_cloud", 1);
  height_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("wall_height_markers", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("ground_plane_marker", 1);

  ros::spin();

  close(sock_fd);

  return 0;
}
