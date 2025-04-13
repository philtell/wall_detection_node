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

#include <map>
#include <vector>
#include <iostream>

struct WallSlice {
    float y_center;
    float z_min;
    float z_max;
    float height() const { return z_max - z_min; }
};


class GroundSegmetation
{
    private:
    // IMU 数据
    double imu_pitch = 0.0;  // 车辆俯仰角度
    double imu_roll = 0.0;   // 车辆横滚角度
    public:
        void ground_sgementation(const sensor_msgs::PointCloud2ConstPtr &input);
        std::vector<WallSlice> sliceWallByY(pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cluster, float slice_width);
        float calculateSlopeAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,float& wall_height_out,pcl::PointCloud<pcl::PointXYZ>::Ptr& wall_cluster_out);
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