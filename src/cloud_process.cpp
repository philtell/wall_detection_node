#include "cloud_process.h"

// 输入：聚类点云
std::vector<WallSlice> GroundSegmentation::sliceWallByY(
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
std::vector<WallSlice> GroundSegmentation::sliceWallByX(
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

// 替代 mqtt_client = new mqtt::async_client(...) 等初始化代码
void GroundSegmentation::initSocket(const std::string &ip, int port)
{
    sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd < 0)
    {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr);

    while (true)
    {
        if (connect(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == 0)
        {
            ROS_INFO("Connected to socket server at %s:%d", ip.c_str(), port);
            break;
        }
        else
        {
            perror("Socket connection failed, retrying in 1s...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

GroundSegmentation::~GroundSegmentation()
{

}

// 发布平面
void GroundSegmentation::publishGroundPlaneMarker(const pcl::ModelCoefficients::Ptr &coefficients,
                                                 ros::Publisher &marker_pub,
                                                 const std::string &frame_id)
{
    // 提取平面参数 ax + by + cz + d = 0
    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];

    // 创建平面 marker（以一个矩形表示）
    visualization_msgs::Marker marker;
    marker.header.frame_id = "perception"; // 或你的坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "ground_plane";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 2.0;
    marker.scale.y = 2.0;
    marker.scale.z = 2.0;

    marker.color.a = 0.5;
    marker.color.r = 0.2;
    marker.color.g = 0.8;
    marker.color.b = 0.2;

    // 构建矩形平面顶点（大小为10m x 10m）
    float size = 15.0;
    std::vector<Eigen::Vector3f> corners;
    corners.emplace_back(-size, 0, 0);
    corners.emplace_back(size, 0, 0);
    corners.emplace_back(size, size * 2, 0);
    corners.emplace_back(-size, size * 2, 0);

    // 将每个角点投影到平面上
    for (auto &pt : corners)
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

GroundSegmentation::GroundSegmentation(ros::NodeHandle &nh) : nh_(nh)
{
    // initSocket("127.0.0.1", 9000); // 你可以替换为目标服务器的 IP 和端口
    loadParams();
    service_ = nh_.advertiseService("reload_params", &GroundSegmentation::reloadCallback, this);
    // ROS 订阅
    cloud_sub = nh_.subscribe("lidar_front/raw_cloud", 10, &GroundSegmentation::cloudCallback,this);
    rtk_sub = nh_.subscribe("rtk_test", 10, &GroundSegmentation::gpsCallback,this);
    imu_sub = nh_.subscribe("imu", 10, &GroundSegmentation::imuCallback,this);
    non_ground_pub = nh_.advertise<sensor_msgs::PointCloud2>("non_ground_cloud", 1);
    ground_pub = nh_.advertise<sensor_msgs::PointCloud2>("transform_ground_cloud", 1);
    height_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("wall_height_markers", 1);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("ground_plane_marker", 1);
}

void GroundSegmentation::gpsCallback(const gps_common::GPSFixConstPtr &msg)
{
    current_lat = msg->latitude;
    current_lon = msg->longitude;
    ROS_INFO("lat: %f, lon: %f", current_lat, current_lon);
}

void GroundSegmentation::imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    std::lock_guard<std::mutex> lock(imu_mutex);
    imu_orientation.w() = imu_msg->orientation.w;
    imu_orientation.x() = imu_msg->orientation.x;
    imu_orientation.y() = imu_msg->orientation.y;
    imu_orientation.z() = imu_msg->orientation.z;
    m_imu_msg = *imu_msg; // 保存最新的 IMU 消息
    // ROS_INFO("IMU orientation updated. w: %f, x: %f, y: %f, z: %f",
    //          imu_orientation.w(), imu_orientation.x(), imu_orientation.y(), imu_orientation.z());
}

void GroundSegmentation::publishToSocket(const WallInfo &info, const pcl::PointCloud<pcl::PointXYZI>::Ptr &wall_cloud)
{
    json j;
    j["wall_points"] = json::array();
    for (const auto &pt : info.wall_points)
    {
        j["wall_points"].push_back({{"x", pt.x}, {"y", pt.y}, {"z", pt.z}});
    }
    j["slope_angle"] = info.back_slope_angle;
    j["lat"] = info.latitude;
    j["lon"] = info.longitude;

    json cloud_json = json::array();
    for (const auto &pt : wall_cloud->points)
    {
        cloud_json.push_back({pt.x, pt.y, pt.z});
    }
    j["cloud"] = cloud_json;

    std::string payload = j.dump();
    int ret = send(sock_fd, payload.c_str(), payload.size(), 0);
    if (ret < 0)
    {
        perror("Socket send failed");
    }
    else
    {
        ROS_INFO("Wall info sent to socket, size: %ld bytes", payload.size());
    }
}

void GroundSegmentation::computeRollPitch(float ax, float ay, float az, float& roll_deg, float& pitch_deg) {
    float roll  = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));
    roll_deg = roll * 180.0 / M_PI;
    pitch_deg = pitch * 180.0 / M_PI;
}


void GroundSegmentation::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_non_ground(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZI>);
    // 记录程序开始时间
    auto start_time = std::chrono::high_resolution_clock::now();

    Eigen::Quaternionf imu_q;
    {
        std::lock_guard<std::mutex> lock(imu_mutex);
        imu_q = imu_orientation;
        current_imu_msg = m_imu_msg;
    }

    // 将点云从车身坐标变换到水平坐标（即去除roll和pitch）
    Eigen::Matrix3f R = imu_q.toRotationMatrix();

    // 提取 roll 和 pitch 的反旋转
    Eigen::Vector3f euler = R.eulerAngles(0, 1, 2); // roll pitch yaw
    float roll = euler[0], pitch = euler[1];
    float ay = current_imu_msg.linear_acceleration.y;
    float ax = current_imu_msg.linear_acceleration.x;
    float az = current_imu_msg.linear_acceleration.z;
    float roll_deg = 0, pitch_deg = 0;
    computeRollPitch(ax, ay, az, roll_deg, pitch_deg);
    std::cout << "Roll: " << roll_deg << "°, Pitch: " << pitch_deg << "°" << std::endl;
    float roll_rad = roll_deg * M_PI / 180.0f;
    float pitch_rad = pitch_deg * M_PI / 180.0f;

    // 逆向旋转构造仿射变换
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(-roll_rad, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(-pitch_rad, Eigen::Vector3f::UnitY()));

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>());
    std::vector<WallSlice> height_slice;
    double slope_angle;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    ground_sgementation(cloud_msg, cloud_non_ground, cloud_ground, height_slice, slope_angle, coefficients);
    sensor_msgs::PointCloud2 cloud_msg2;
    pcl::toROSMsg(*cloud_non_ground, cloud_msg2);
    cloud_msg2.header.frame_id = "perception"; // 或你的坐标系
    cloud_msg2.header.stamp = ros::Time::now();
    non_ground_pub.publish(cloud_msg2);
    pcl::transformPointCloud(*cloud_ground, *cloud_out, transform);
    ROS_INFO("ground points size:%ld",cloud_out->points.size());
    // 发布矫正后的点云（可选）
    sensor_msgs::PointCloud2 corrected_msg;
    pcl::toROSMsg(*cloud_out, corrected_msg);
    corrected_msg.header = cloud_msg->header;
    ground_pub.publish(corrected_msg);
    double transform_angle = slope_angle;
    getSlopAngle(cloud_out, transform_angle);
    ROS_INFO("roll: %.3f pitch:%.3f,before angle:%.3f after transform angle:%.3f", roll_deg, pitch_deg, slope_angle, transform_angle);
    publishGroundPlaneMarker(coefficients, marker_pub);
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    WallInfo info;
    info.wall_points.clear();
    for (const auto &slice : height_slice)
    {
        WallPoint pt;
        pt.x = slice.x_center; // 挡墙 X 位置固定
        pt.y = slice.y_center; // Y 中心
        pt.z = slice.height(); // Z 为高度
        info.wall_points.push_back(pt);
        visualization_msgs::Marker text;
        text.header.frame_id = "perception";
        text.header.stamp = ros::Time::now();
        text.ns = "wall_heights";
        text.id = id++;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;

        text.pose.position.x = slice.x_center; // 固定在墙体 x 附近
        text.pose.position.y = slice.y_center;
        text.pose.position.z = slice.z_max + 0.3; // 提高显示位置
        text.pose.orientation.w = 1.0;

        text.scale.z = 0.4; // 字体高度
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

    info.back_slope_angle = slope_angle; // 这里只是示例，实际需要计算
    info.latitude = current_lat;
    info.longitude = current_lon;

    publishToSocket(info, cloud_non_ground);

    // publishToMQTT(info, cloud_non_ground);

    // 程序退出前，记录结束时间
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    ROS_INFO_STREAM("Cost Time: " << duration << " ms");
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> GroundSegmentation::clusterByGridBFS(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud)
{
    const float grid_size = 1.0;
    const float x_min = this->roi_min_x_, x_max = this->roi_max_x_;
    const float y_min = this->roi_min_y_, y_max = this->roi_max_y_;

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

// float GroundSegmentation::calculateSlopeAngle(
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
//     float &wall_height_out,
//     pcl::PointCloud<pcl::PointXYZ>::Ptr &wall_cluster_out)
// {
//     // 1. 地面分割
//     pcl::SACSegmentation<pcl::PointXYZ> seg;
//     pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);
//     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

//     seg.setOptimizeCoefficients(true);
//     seg.setModelType(pcl::SACMODEL_PLANE);
//     seg.setMethodType(pcl::SAC_RANSAC);
//     seg.setDistanceThreshold(0.1);
//     seg.setInputCloud(cloud_in);
//     seg.segment(*ground_inliers, *coefficients);

//     if (ground_inliers->indices.empty())
//     {
//         ROS_WARN("No ground found.");
//         return -1;
//     }

//     // 2. 提取非地面点
//     pcl::ExtractIndices<pcl::PointXYZ> extract;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonground(new pcl::PointCloud<pcl::PointXYZ>);
//     extract.setInputCloud(cloud_in);
//     extract.setIndices(ground_inliers);
//     extract.setNegative(true); // 提取非地面
//     extract.filter(*cloud_nonground);

//     // 3. 欧几里得聚类
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//     tree->setInputCloud(cloud_nonground);

//     std::vector<pcl::PointIndices> cluster_indices;
//     pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//     ec.setClusterTolerance(0.3); // 可调
//     ec.setMinClusterSize(30);
//     ec.setMaxClusterSize(100000);
//     ec.setSearchMethod(tree);
//     ec.setInputCloud(cloud_nonground);
//     ec.extract(cluster_indices);

//     // 4. 筛选区域并找最大聚类
//     int max_points = 0;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr selected_cluster(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointIndices max_indexs;
//     for (const auto &indices : cluster_indices)
//     {
//         // pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
//         // for (int idx : indices.indices)
//         //     cluster->points.push_back(cloud_nonground->points[idx]);

//         if ( indices.indices.size() > max_points)
//         {
//             // *selected_cluster = *cluster;
//             max_points = indices.indices.size();
//             max_indexs = indices;
//         }
//     }
//     if(max_indexs.indices.empty())
//     {
//         ROS_WARN("No wall cluster found.");
//         return -1;
//     }
//     for(int idx : max_indexs.indices)
//     {
//         selected_cluster->points.push_back(cloud_nonground->points[idx]);
//     }

//     // 5. 计算挡墙高度：z 最大 - z 最小
//     float min_z = std::numeric_limits<float>::max();
//     float max_z = -std::numeric_limits<float>::max();

//     for (const auto &pt : selected_cluster->points)
//     {
//         if (pt.z < min_z)
//             min_z = pt.z;
//         if (pt.z > max_z)
//             max_z = pt.z;
//     }

//     wall_height_out = max_z - min_z;
//     wall_cluster_out = selected_cluster;
//     return 0;
// }
void GroundSegmentation::getSlopAngle(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ground, double &slope_angle)
{
    if (cloud_ground->empty())
    {
        return;
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

void GroundSegmentation::ground_sgementation(const sensor_msgs::PointCloud2ConstPtr &input, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_non_ground, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ground, std::vector<WallSlice> &height_slice, double &slope_angle, pcl::ModelCoefficients::Ptr &coefficients)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
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
        if (point.x < this->roi_max_x_ && point.x > this->roi_min_x_ && point.y < this->roi_max_y_ && point.y > this->roi_min_y_)
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
