#include "cloud_process.h"

// 输入：聚类点云
std::vector<WallSlice> GroundSegmetation::sliceWallByY(pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cluster, float slice_width = 1.0f)
{
    std::map<int, std::vector<float>> slice_zs;

    // 将点按 y 分组（每 1m 为一个 slice）
    for (const auto& pt : wall_cluster->points) {
        int y_index = static_cast<int>(pt.y / slice_width);
        slice_zs[y_index].push_back(pt.z);
    }

    // 计算每个 slice 的高度信息
    std::vector<WallSlice> slices;
    for (const auto& [index, zs] : slice_zs) {
        if (zs.size() < 3) continue;  // 过滤掉太小的片段
        float min_z = *std::min_element(zs.begin(), zs.end());
        float max_z = *std::max_element(zs.begin(), zs.end());

        WallSlice slice;
        slice.y_center = (index + 0.5f) * slice_width;
        slice.z_min = min_z;
        slice.z_max = max_z;
        slices.push_back(slice);
    }

    return slices;
}

float GroundSegmetation::calculateSlopeAngle(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
    float& wall_height_out,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& wall_cluster_out)
{
    // 1. 地面分割
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);
    seg.setInputCloud(cloud_in);
    seg.segment(*ground_inliers, *coefficients);

    if (ground_inliers->indices.empty()) {
        ROS_WARN("No ground found.");
        return -1;
    }

    // 2. 提取非地面点
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonground(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud_in);
    extract.setIndices(ground_inliers);
    extract.setNegative(true);  // 提取非地面
    extract.filter(*cloud_nonground);

    // 3. 欧几里得聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_nonground);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.3);  // 可调
    ec.setMinClusterSize(30);
    ec.setMaxClusterSize(100000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_nonground);
    ec.extract(cluster_indices);

    // 4. 筛选区域并找最大聚类
    int max_points = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (int idx : indices.indices)
            cluster->points.push_back(cloud_nonground->points[idx]);

        // 判断是否在指定区域
        bool in_range = false;
        for (const auto& pt : cluster->points) {
            if (pt.x > -22 && pt.x < -18 && pt.y > 7 && pt.y < 20) {
                in_range = true;
                break;
            }
        }

        if (in_range && cluster->points.size() > max_points) {
            *selected_cluster = *cluster;
            max_points = cluster->points.size();
        }
    }

    if (selected_cluster->empty()) {
        ROS_WARN("No wall cluster found in specified region.");
        return -1;
    }

    // 5. 计算挡墙高度：z 最大 - z 最小
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for (const auto& pt : selected_cluster->points) {
        if (pt.z < min_z) min_z = pt.z;
        if (pt.z > max_z) max_z = pt.z;
    }

    wall_height_out = max_z - min_z;
    wall_cluster_out = selected_cluster;
    return 0;
}

void GroundSegmetation::ground_sgementation(const sensor_msgs::PointCloud2ConstPtr& input,pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_non_ground,std::vector<WallSlice>& height_slice,double& slope_angle)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZI>);
    // 转换 PointCloud2 数据为 PCL 点云
    pcl::fromROSMsg(*input, *cloud);
    // cloud_filtered->reserve(cloud->points.size());
    // for(auto point : cloud->points)
    // {
    //     if(point.x > 15 && point.x <-15 && point.z <5 && point.z  >-5 && point.y > 7 && point.y < 40)
    //     {
    //         cloud_filtered->points.push_back(point);
    //     }
    // }

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

    // // 提取出地面点
    // pcl::ExtractIndices<pcl::PointXYZI> extract;
    // extract.setInputCloud(cloud_filtered);
    // extract.setIndices(inlier_indices);
    // extract.setNegative(false);  // 提取平面内的点
    // extract.filter(*cloud_ground);

    // // 提取非地面点
    // extract.setNegative(true);  // 提取平面外的点
    // extract.filter(*cloud_non_ground);

    // height_slice = sliceWallByY(cloud_non_ground);

    // // 计算反坡角度
    // slope_angle = calculateSlopeAngle(coefficients);
    
    // // 调整反坡角度，考虑IMU的俯仰角度
    // double corrected_slope_angle = slope_angle - imu_pitch; // 校正反坡角度

    // ROS_INFO("Ground points: %zu", cloud_ground->points.size());
    // ROS_INFO("Non-ground points: %zu", cloud_non_ground->points.size());
    // ROS_INFO("Original slope angle: %.2f degrees", slope_angle);
    // ROS_INFO("Corrected slope angle (adjusted for pitch): %.2f degrees", corrected_slope_angle);
}
