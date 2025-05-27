#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath> // for acos() and sqrt()
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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
#include <queue>
#include <unordered_set>
#include <map>
#include <vector>
#include <ros/ros.h>
#include <std_srvs/Trigger.h> // 标准触发型Service
#include <iostream>
const float grid_size = 0.5;
const float x_min = -20.0f;
const float x_max = 20.0f;
const float y_min = 5.0f;
const float y_max = 40.0f;
const int grid_width = static_cast<int>((x_max - x_min) / grid_size);
const int grid_height = static_cast<int>((y_max - y_min) / grid_size);


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






struct WallSlice
{
    float x_center = -20.0f; // 固定或通过聚类确定
    float y_center;
    float z_min;
    float z_max;
    float height() const { return z_max - z_min; }
};

class GroundSegmentation {
public:
  ~GroundSegmentation();
private:
    // IMU 数据
    double imu_pitch = 0.0; // 车辆俯仰角度
    double imu_roll = 0.0;  // 车辆横滚角度
    ros::NodeHandle nh_;
    ros::ServiceServer service_;
    int sock_fd;
    double current_lat = 0.0;
    double current_lon = 0.0;
    Eigen::Quaternionf imu_orientation = Eigen::Quaternionf::Identity();
    sensor_msgs::Imu m_imu_msg,current_imu_msg;
    std::mutex imu_mutex;
    struct sockaddr_in server_addr;

    // 参数变量
    std::string robot_name_;
    int robot_id_;
    double robot_speed_;
    double roi_min_x_;
    double roi_max_x_;
    double roi_min_y_;
    double roi_max_y_;

    ros::Publisher non_ground_pub;
    ros::Publisher adjust_ground_pub;
    ros::Publisher ground_pub;
    ros::Publisher height_marker_pub;
    ros::Publisher marker_pub;
    ros::Subscriber cloud_sub;
    ros::Subscriber rtk_sub;
    ros::Subscriber imu_sub;
    void initSocket(const std::string& ip, int port);
public:
    GroundSegmentation(ros::NodeHandle &nh);

    void loadParams()
    {
        nh_.param<std::string>("robot_name", robot_name_, "default_robot");
        nh_.param<int>("robot_id", robot_id_, 0);
        nh_.param<double>("robot_speed", robot_speed_, 1.0);
        nh_.param<double>("min_x", this->roi_min_x_, -35.0);
        nh_.param<double>("max_x", this->roi_max_x_, 0.0);
        nh_.param<double>("min_y", this->roi_min_y_, 15.0);
        nh_.param<double>("max_y", this->roi_max_y_, 65.0);
        ROS_INFO("robot_name: %s, robot_id: %d, robot_speed: %f, min_x: %f, max_x: %f, min_y: %f, max_y: %f",
                 robot_name_.c_str(), robot_id_, robot_speed_, roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_);
    }
    void rotatePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out,
                      const Eigen::Matrix3f& R);
    void computePlaneNormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_cloud, Eigen::Vector3f& normal);
    void gpsCallback(const gps_common::GPSFixConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    Eigen::Matrix3f computeRotationToHorizontal(const Eigen::Vector3f& normal);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void publishGroundPlaneMarker(const pcl::ModelCoefficients::Ptr& coefficients,
                              ros::Publisher& marker_pub,
                              const std::string& frame_id = "map");
    void publishToSocket(const WallInfo& info, const pcl::PointCloud<pcl::PointXYZI>::Ptr& wall_cloud);
    void computeRollPitch(float ax, float ay, float az, float& roll_deg, float& pitch_deg);

    bool reloadCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        loadParams();
        res.success = true;
        res.message = "Parameters reloaded successfully.";
        return true;
    }
    void ground_sgementation(const sensor_msgs::PointCloud2ConstPtr &input, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_non_ground, pcl::PointCloud<pcl::PointXYZI>::Ptr &non_ground, std::vector<WallSlice> &height_slice, double &slope_angle, pcl::ModelCoefficients::Ptr &coefficients);
    void getSlopAngle(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ground, double &slope_angle);
    std::vector<WallSlice> sliceWallByX(pcl::PointCloud<pcl::PointXYZI>::Ptr wall_cluster, float slice_width);
    std::vector<WallSlice> sliceWallByY(pcl::PointCloud<pcl::PointXYZI>::Ptr wall_cluster, float slice_width);

    // float calculateSlopeAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float &wall_height_out, pcl::PointCloud<pcl::PointXYZ>::Ptr &wall_cluster_out);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusterByGridBFS(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud);
    // 计算平面法向量与水平面之间的夹角（反坡）
    double calculateSlopeAngle(const pcl::ModelCoefficients::Ptr &coefficients)
    {
        double A = coefficients->values[0];
        double B = coefficients->values[1];
        double C = coefficients->values[2];

        double angle_rad = acos(C / sqrt(A * A + B * B + C * C));
        double angle_deg = angle_rad * 180.0 / M_PI;

        return angle_deg;
    }
};