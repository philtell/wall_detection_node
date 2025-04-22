#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>  // for acos() and sqrt()
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <queue>
#include <unordered_set>
#include <map>
#include <vector>
#include <iostream>
const float grid_size = 0.5;
const float x_min = -20.0f;
const float x_max = 20.0f;
const float y_min = 5.0f;
const float y_max = 40.0f;
const int grid_width = static_cast<int>((x_max - x_min) / grid_size);
const int grid_height = static_cast<int>((y_max - y_min) / grid_size);

struct WallSlice {
    float x_center = -20.0f;   // 固定或通过聚类确定
    float y_center;
    float z_min;
    float z_max;
    float height() const { return z_max - z_min; }
};

class GroundSegmetation
{
private:
    // IMU 数据
    double imu_pitch = 0.0; // 车辆俯仰角度
    double imu_roll = 0.0;  // 车辆横滚角度
public:
    GroundSegmetation()
    {

    }
    void ground_sgementation(const sensor_msgs::PointCloud2ConstPtr& input,pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_non_ground,std::vector<WallSlice>& height_slice,double& slope_angle,pcl::ModelCoefficients::Ptr &coefficients);
    std::vector<WallSlice> sliceWallByX(pcl::PointCloud<pcl::PointXYZI>::Ptr wall_cluster, float slice_width);
    float calculateSlopeAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float &wall_height_out, pcl::PointCloud<pcl::PointXYZ>::Ptr &wall_cluster_out);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusterByGridBFS(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud);
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