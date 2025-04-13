#include "cloud_process.h"

void GroundSegmetation::ground_sgementation(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_non_ground(new pcl::PointCloud<pcl::PointXYZI>);
    // 转换 PointCloud2 数据为 PCL 点云
    pcl::fromROSMsg(*input, *cloud);

    // 过滤掉过远和过近的点（假设地面在一定的Z范围内）
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-5.0, 5.0); // 设置Z轴过滤范围
    pass.filter(*cloud_filtered);

    // 创建一个 RANSAC 分割对象
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);  // 设置模型为平面
    seg.setMethodType(pcl::SAC_RANSAC);     // 设置方法为RANSAC
    seg.setDistanceThreshold(0.2);          // 设置阈值，控制点与平面的最大距离

    // 输入点云
    seg.setInputCloud(cloud_filtered);

    // 输出
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inlier_indices(new pcl::PointIndices);

    // 进行地面分割，返回平面内的点
    seg.segment(*inlier_indices, *coefficients);

    if (inlier_indices->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return;
    }

    // 提取出地面点
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inlier_indices);
    extract.setNegative(false);  // 提取平面内的点
    extract.filter(*cloud_ground);

    // 提取非地面点
    extract.setNegative(true);  // 提取平面外的点
    extract.filter(*cloud_non_ground);


    // 计算反坡角度
    double slope_angle = calculateSlopeAngle(coefficients);
    
    // 调整反坡角度，考虑IMU的俯仰角度
    double corrected_slope_angle = slope_angle - imu_pitch; // 校正反坡角度

    ROS_INFO("Ground points: %zu", cloud_ground->points.size());
    ROS_INFO("Non-ground points: %zu", cloud_non_ground->points.size());
    ROS_INFO("Original slope angle: %.2f degrees", slope_angle);
    ROS_INFO("Corrected slope angle (adjusted for pitch): %.2f degrees", corrected_slope_angle);
}
