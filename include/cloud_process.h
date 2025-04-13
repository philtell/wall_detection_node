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


class GroundSegmetation
{
    private:
    // IMU 数据
    double imu_pitch = 0.0;  // 车辆俯仰角度
    double imu_roll = 0.0;   // 车辆横滚角度
    public:
        void ground_sgementation(const sensor_msgs::PointCloud2ConstPtr &input);
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