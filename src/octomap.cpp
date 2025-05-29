#include "xvins_octomap/octomap.hpp"
#include <pcl_conversions/pcl_conversions.h>





void OctomapMapper::initMap(std::shared_ptr<rclcpp::Node> node) {
    // node
    node_ = node;

    // load param
    node_->declare_parameter("octomap.resolution", 0.2);
    node_->declare_parameter("octomap.prob_hit", 0.7);
    node_->declare_parameter("octomap.prob_miss", 0.4);
    node_->declare_parameter("octomap.occupancy_thresh", 0.5);
    node_->declare_parameter("octomap.pointcloudMaxX", 2.5);
    node_->declare_parameter("octomap.pointcloudMinX", -2.5);
    node_->declare_parameter("octomap.pointcloudMaxY", 2.5);
    node_->declare_parameter("octomap.pointcloudMinY", -2.5);
    node_->declare_parameter("octomap.pointcloudMaxZ", 2.0);
    node_->declare_parameter("octomap.pointcloudMinZ", -2.0);
    node_->declare_parameter("octomap.m_compressMap", false);

    node_->get_parameter("octomap.resolution", mp_.resolution);
    node_->get_parameter("octomap.prob_hit", mp_.prob_hit);
    node_->get_parameter("octomap.prob_miss", mp_.prob_miss);
    node_->get_parameter("octomap.occupancy_thresh", mp_.occupancy_thresh);
    node_->get_parameter("octomap.pointcloudMaxX", mp_.m_pointcloudMaxX);
    node_->get_parameter("octomap.pointcloudMinX", mp_.m_pointcloudMinX);
    node_->get_parameter("octomap.pointcloudMaxY", mp_.m_pointcloudMaxY);
    node_->get_parameter("octomap.pointcloudMinY", mp_.m_pointcloudMinY);
    node_->get_parameter("octomap.pointcloudMaxZ", mp_.m_pointcloudMaxZ);
    node_->get_parameter("octomap.pointcloudMinZ", mp_.m_pointcloudMinZ);
    node_->get_parameter("octomap.m_compressMap", mp_.m_compressMap);

    // init OctoMap
    m_octree_ = std::make_shared<octomap::OcTree>(mp_.resolution);
    m_octree_->setProbHit(mp_.prob_hit);
    m_octree_->setProbMiss(mp_.prob_miss);
    m_octree_->setOccupancyThres(mp_.occupancy_thresh);


    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud>(
        "/point_cloud", 10, std::bind(&OctomapMapper::cloudCallback, this, std::placeholders::_1));

    pose_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, std::bind(&OctomapMapper::poseCallback, this, std::placeholders::_1));

    octomap_pub_ = node_->create_publisher<octomap_msgs::msg::Octomap>("cotomap_binary", 10);

}

std::shared_ptr<octomap::OcTree> OctomapMapper::getMap() const{
    return m_octree_;
}

void OctomapMapper::cloudCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg){
    // 1. Convert ros messages to pcl messages
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& point : msg->points) {
        cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
    }
    // 2. filter
    auto pc_cloud = filterPointCloud(cloud);

    // 3. Convert pcl messages to octomap messages
    octomap::Pointcloud octomap_cloud;
    for (const auto& point : pc_cloud->points)
    {
        octomap_cloud.push_back(octomap::point3d(point.x, point.y, point.z));
    }
     // Insert point cloud data into OctoMap
     // TODO: 1. Set max(x,y,z) point cloud filter 2. New octomap insertion method 3. Subscribe to xvins imu pose as origin and pass it into m_octree_
     
     // m_octree_->insertPointCloud(octomap_cloud, sensor_origin_);

     // Use ray tracing to update octomap
     insertPointCloud(octomap_cloud);

     publishMap();
}

void OctomapMapper::poseCallback(const nav_msgs::msg::Path::SharedPtr msg){
     // Extract the latest pose (position) from the path
     if (!msg->poses.empty()) {
        sensor_origin_ = octomap::point3d(msg->poses.back().pose.position.x, 
                                          msg->poses.back().pose.position.y,
                                          msg->poses.back().pose.position.z);
    }else{
        RCLCPP_ERROR(node_->get_logger(), "Receive pose wrong");
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr OctomapMapper::filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    const float min_x = sensor_origin_.x() + mp_.m_pointcloudMinX;
    const float max_x = sensor_origin_.x() + mp_.m_pointcloudMaxX;
    const float min_y = sensor_origin_.y() + mp_.m_pointcloudMinY;
    const float max_y = sensor_origin_.y() + mp_.m_pointcloudMaxY;
    const float min_z = sensor_origin_.z() + mp_.m_pointcloudMinZ;
    const float max_z = sensor_origin_.z() + mp_.m_pointcloudMaxZ;

    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(min_x, max_x); 
    pass_x.setInputCloud(cloud);
    pass_x.filter(*filtered_cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(min_y, max_y);
    pass_y.setInputCloud(filtered_cloud);
    pass_y.filter(*filtered_cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(min_z, max_z); 
    pass_z.setInputCloud(filtered_cloud);
    pass_z.filter(*filtered_cloud);

    

    return filtered_cloud;
}

inline void OctomapMapper::updateMinKey(const octomap::OcTreeKey& in,
    octomap::OcTreeKey& min) {
        for (unsigned i = 0; i < 3; ++i) {
        min[i] = std::min(in[i], min[i]);
        }
    }

inline void OctomapMapper::updateMaxKey(const octomap::OcTreeKey& in,
    octomap::OcTreeKey& max) {
    for (unsigned i = 0; i < 3; ++i) {
    max[i] = std::max(in[i], max[i]);
    }
}

void OctomapMapper::insertPointCloud(octomap::Pointcloud &octomap_cloud){
    for (const auto& point : octomap_cloud) {
        // Use ray tracing to insert free space
        if (m_octree_->computeRayKeys(sensor_origin_, point, m_keyRay)) {
            free_cells.insert(m_keyRay.begin(), m_keyRay.end());
        }

        // Check if the point is occupied
        octomap::OcTreeKey key;
        if (m_octree_->coordToKeyChecked(point, key)) {
            occupied_cells.insert(key);
            updateMinKey(key, m_updateBBXMin);
            updateMaxKey(key, m_updateBBXMax);
        }
    }

    // Mark free cells only if not seen as occupied
    for (auto it = free_cells.begin(), end = free_cells.end(); it != end; ++it) {
        if (occupied_cells.find(*it) == occupied_cells.end()) {
            m_octree_->updateNode(*it, false);  // Mark as free
        }
    }

    // Mark all occupied cells
    for (auto it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it) {
        m_octree_->updateNode(*it, true);  // Mark as occupied
    }

    // Optionally prune the octree 
    if (mp_.m_compressMap) {
        m_octree_->prune();
    }
}

void OctomapMapper::publishMap()
{
    octomap_msgs::msg::Octomap map;
    map.header.stamp = node_->get_clock()->now();
    map.header.frame_id = "world"; 
    if (octomap_msgs::binaryMapToMsg(*m_octree_, map)) {
        // Publishing OctoMap messages
        RCLCPP_INFO(node_->get_logger(), "publish octobinaryMap!");
        octomap_pub_->publish(map);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Error serializing OctoMap");
    }
}