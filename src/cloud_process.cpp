#include "cloud_process.h"

// 输入：聚类点云
std::vector<WallSlice> GroundSegmetation::sliceWallByY(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster,
    float slice_width)
{
    std::vector<WallSlice> slices;

    if (cluster->empty())
        return slices;

    float y_min = std::numeric_limits<float>::max();
    float y_max = std::numeric_limits<float>::lowest();

    for (const auto &pt : cluster->points)
    {
        if (pt.y < y_min)
            y_min = pt.y;
        if (pt.y > y_max)
            y_max = pt.y;
    }

    int num_slices = static_cast<int>((y_max - y_min) / slice_width) + 1;
    std::vector<std::vector<pcl::PointXYZI>> bins(num_slices);

    for (const auto &pt : cluster->points)
    {
        int idx = static_cast<int>((pt.y - y_min) / slice_width);
        if (idx >= 0 && idx < num_slices)
            bins[idx].push_back(pt);
    }

    for (int i = 0; i < num_slices; ++i)
    {
        if (bins[i].empty())
            continue;

        float z_min = std::numeric_limits<float>::max();
        float z_max = std::numeric_limits<float>::lowest();
        float y_sum = 0.0f;
        float x_sum = 0.0f;

        for (const auto &pt : bins[i])
        {
            if (pt.z < z_min)
                z_min = pt.z;
            if (pt.z > z_max)
                z_max = pt.z;
            y_sum += pt.y;
            x_sum += pt.x;
        }

        WallSlice slice;
        slice.x_center = x_sum / bins[i].size();
        slice.y_center = y_sum / bins[i].size();
        slice.z_min = z_min;
        slice.z_max = z_max;

        slices.push_back(slice);
    }

    return slices;
}

// 输入：聚类点云
std::vector<WallSlice> GroundSegmetation::sliceWallByX(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster,
    float slice_width)
{
    std::vector<WallSlice> slices;

    if (cluster->empty())
        return slices;

    float x_min = std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::lowest();

    for (const auto &pt : cluster->points)
    {
        if (pt.x < x_min)
            x_min = pt.x;
        if (pt.x > x_max)
            x_max = pt.x;
    }

    int num_slices = static_cast<int>((x_max - x_min) / slice_width) + 1;
    std::vector<std::vector<pcl::PointXYZI>> bins(num_slices);

    for (const auto &pt : cluster->points)
    {
        int idx = static_cast<int>((pt.x - x_min) / slice_width);
        if (idx >= 0 && idx < num_slices)
            bins[idx].push_back(pt);
    }

    for (int i = 0; i < num_slices; ++i)
    {
        if (bins[i].empty())
            continue;

        float z_min = std::numeric_limits<float>::max();
        float z_max = std::numeric_limits<float>::lowest();
        float y_sum = 0.0f;
        float x_sum = 0.0f;

        for (const auto &pt : bins[i])
        {
            if (pt.z < z_min)
                z_min = pt.z;
            if (pt.z > z_max)
                z_max = pt.z;
            y_sum += pt.y;
            x_sum += pt.x;
        }

        WallSlice slice;
        slice.x_center = x_sum / bins[i].size();
        slice.y_center = y_sum / bins[i].size();
        slice.z_min = z_min;
        slice.z_max = z_max;

        slices.push_back(slice);
    }

    return slices;
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> GroundSegmetation::clusterByGridBFS(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud)
{
    const float grid_size = 1.0;
    const float x_min = -30.0f, x_max = 10.0f;
    const float y_min = 15.0f, y_max = 65.0f;

    const int grid_cols = static_cast<int>((x_max - x_min) / grid_size); // X方向
    const int grid_rows = static_cast<int>((y_max - y_min) / grid_size); // Y方向

    // 建立格子索引：每个格子里存放的是点的索引
    std::vector<std::vector<std::vector<int>>> grid(grid_rows, std::vector<std::vector<int>>(grid_cols));

    int valid_points = 0;
    int no_valid_points = 0;
    // 映射点到栅格
    for (size_t i = 0; i < input_cloud->points.size(); ++i)
    {
        const auto &pt = input_cloud->points[i];
        if (pt.x < x_min || pt.x >= x_max || pt.y < y_min || pt.y >= y_max)
        {
            // ROS_INFO("Point %zu: x = %.2f, y = %.2f", i, pt.x, pt.y);
            ++no_valid_points;
            continue;
        }

        ++valid_points;
        int col = static_cast<int>((pt.x - x_min) / grid_size);
        int row = static_cast<int>((pt.y - y_min) / grid_size);
        grid[row][col].push_back(i);
    }
    ROS_INFO("valid points: %d no valid points: %d", valid_points, no_valid_points);

    int non_empty_cells = 0;
    for (int r = 0; r < grid_rows; ++r)
    {
        for (int c = 0; c < grid_cols; ++c)
        {
            if (!grid[r][c].empty())
                ++non_empty_cells;
        }
    }
    ROS_INFO("no empty grid size: %d", non_empty_cells);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    std::vector<std::vector<bool>> visited(grid_rows, std::vector<bool>(grid_cols, false));

    const int dx[4] = {-1, 0, 1, 0};
    const int dy[4] = {0, 1, 0, -1};

    // BFS 遍历每个格子
    for (int r = 0; r < grid_rows; ++r)
    {
        for (int c = 0; c < grid_cols; ++c)
        {
            if (visited[r][c] || grid[r][c].empty())
                continue;

            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
            std::queue<std::pair<int, int>> q;
            q.emplace(r, c);
            visited[r][c] = true;

            while (!q.empty())
            {
                auto [cr, cc] = q.front();
                q.pop();

                for (int idx : grid[cr][cc])
                {
                    cluster->points.push_back(input_cloud->points[idx]);
                }

                for (int i = 0; i < 4; ++i)
                {
                    int nr = cr + dy[i];
                    int nc = cc + dx[i];
                    if (nr >= 0 && nr < grid_rows && nc >= 0 && nc < grid_cols &&
                        !visited[nr][nc] && !grid[nr][nc].empty())
                    {
                        q.emplace(nr, nc);
                        visited[nr][nc] = true;
                    }
                }
            }
            ROS_INFO("inner Cluster size: %zu", cluster->points.size());
            if (cluster->points.size() >= 100)
            { // 设定最小聚类点数
                cluster->width = cluster->points.size();
                cluster->height = 1;
                cluster->is_dense = true;
                clusters.push_back(cluster);
            }
        }
    }
    return clusters;
}

float GroundSegmetation::calculateSlopeAngle(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
    float &wall_height_out,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &wall_cluster_out)
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

    if (ground_inliers->indices.empty())
    {
        ROS_WARN("No ground found.");
        return -1;
    }

    // 2. 提取非地面点
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonground(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud_in);
    extract.setIndices(ground_inliers);
    extract.setNegative(true); // 提取非地面
    extract.filter(*cloud_nonground);

    // 3. 欧几里得聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_nonground);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.3); // 可调
    ec.setMinClusterSize(30);
    ec.setMaxClusterSize(100000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_nonground);
    ec.extract(cluster_indices);

    // 4. 筛选区域并找最大聚类
    int max_points = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &indices : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (int idx : indices.indices)
            cluster->points.push_back(cloud_nonground->points[idx]);

        // 判断是否在指定区域
        bool in_range = false;
        for (const auto &pt : cluster->points)
        {
            if (pt.x > -22 && pt.x < -18 && pt.y > 7 && pt.y < 20)
            {
                in_range = true;
                break;
            }
        }

        if (in_range && cluster->points.size() > max_points)
        {
            *selected_cluster = *cluster;
            max_points = cluster->points.size();
        }
    }

    if (selected_cluster->empty())
    {
        ROS_WARN("No wall cluster found in specified region.");
        return -1;
    }

    // 5. 计算挡墙高度：z 最大 - z 最小
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for (const auto &pt : selected_cluster->points)
    {
        if (pt.z < min_z)
            min_z = pt.z;
        if (pt.z > max_z)
            max_z = pt.z;
    }

    wall_height_out = max_z - min_z;
    wall_cluster_out = selected_cluster;
    return 0;
}
void GroundSegmetation::getSlopAngle(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ground, double &slope_angle)
{
    if(cloud_ground->empty())
    {
        return ;
    }

    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE); // 设置模型为平面
    seg.setMethodType(pcl::SAC_RANSAC);    // 设置方法为RANSAC
    seg.setDistanceThreshold(0.2);         // 设置阈值，控制点与平面的最大距离

    // 输入点云
    seg.setInputCloud(cloud_ground);

    // 输出
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inlier_indices(new pcl::PointIndices);

    // 进行地面分割，返回平面内的点
    seg.segment(*inlier_indices, *coefficients);

    slope_angle = calculateSlopeAngle(coefficients);

}

void GroundSegmetation::ground_sgementation(const sensor_msgs::PointCloud2ConstPtr &input, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_non_ground,pcl::PointCloud<pcl::PointXYZI>::Ptr &non_ground, std::vector<WallSlice> &height_slice, double &slope_angle, pcl::ModelCoefficients::Ptr &coefficients)
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
    for (auto point : cloud->points)
    {
        if (point.x < this->roi_max_x_ && point.x > this->roi_min_x_ 
            && point.y < this->roi_max_y_ && point.y > this->roi_min_y_)
        {
            cloud_filtered->points.push_back(point);
        }


        
    }

    // // 过滤掉过远和过近的点（假设地面在一定的Z范围内）
    // pcl::PassThrough<pcl::PointXYZI> pass;
    // pass.setInputCloud(cloud);
    // // pass.setFilterFieldName("z");
    // // pass.setFilterLimits(-5.0, 5.0); // 设置Z轴过滤范围
    // pass.setFilterFieldName("x");
    // pass.setFilterLimits(-30.0, 30.0); // 设置X轴过滤范围
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(15.0, 55.0); // 设置Y轴过滤范围
    // pass.filter(*cloud_filtered);

    // 创建一个 RANSAC 分割对象
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE); // 设置模型为平面
    seg.setMethodType(pcl::SAC_RANSAC);    // 设置方法为RANSAC
    seg.setDistanceThreshold(0.2);         // 设置阈值，控制点与平面的最大距离

    // 输入点云
    seg.setInputCloud(cloud_filtered);

    // 输出
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
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
    extract.setNegative(false); // 提取平面内的点
    extract.filter(*cloud_ground);

    // 提取非地面点
    extract.setNegative(true); // 提取平面外的点
    extract.filter(*cloud_non_ground);
    ROS_INFO("cloud_non_ground: %zu", cloud_non_ground->size());
    height_slice.clear();
    auto wall_clusters = clusterByGridBFS(cloud_non_ground);
    ROS_INFO("Wall clusters: %zu", wall_clusters.size());
    cloud_non_ground->clear();
    // 计算每个聚类的高度
    for (auto cluster : wall_clusters)
    {
        *cloud_non_ground += *cluster;
        auto slices = sliceWallByY(cluster, 3.0f); // 每 1 米切片
        // 收集 slices
        height_slice.insert(height_slice.end(), slices.begin(), slices.end());
    }
    // 计算反坡角度
    slope_angle = calculateSlopeAngle(coefficients);

    // 调整反坡角度，考虑IMU的俯仰角度
    double corrected_slope_angle = slope_angle - imu_pitch; // 校正反坡角度

    ROS_INFO("Ground points: %zu", cloud_ground->points.size());
    ROS_INFO("Non-ground points: %zu", cloud_non_ground->points.size());
    ROS_INFO("Original slope angle: %.2f degrees", slope_angle);
    ROS_INFO("Corrected slope angle (adjusted for pitch): %.2f degrees", corrected_slope_angle);
}
