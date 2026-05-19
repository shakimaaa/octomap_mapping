#include "octomap_mapping/octomap.hpp"
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>





void OctomapMapper::initMap(std::shared_ptr<rclcpp::Node> node) {
    // node
    node_ = node;

    // load param
    node_->declare_parameter("octomap.resolution", 0.3); //0.18 0.05
    node_->declare_parameter("octomap.prob_hit", 0.7);
    node_->declare_parameter("octomap.prob_miss", 0.35);
    node_->declare_parameter("octomap.occupancy_thresh", 0.8);
    node_->declare_parameter("octomap.localmap_thresh", 5.0);
    node_->declare_parameter("octomap.max_range", -1.0);
    node_->declare_parameter("octomap.pointcloudMaxX", 4.0);
    node_->declare_parameter("octomap.pointcloudMinX", -4.0);
    node_->declare_parameter("octomap.pointcloudMaxY", 1.5);
    node_->declare_parameter("octomap.pointcloudMinY", -1.5);
    node_->declare_parameter("octomap.pointcloudMaxZ", 1.5);
    node_->declare_parameter("octomap.pointcloudMinZ", -1.5);
    node_->declare_parameter("octomap.m_compressMap", true);
    node_->declare_parameter("octomap.m_Expansion_range_x", 0.6); 
    node_->declare_parameter("octomap.m_Expansion_range_y", 0.6);
    node_->declare_parameter("octomap.m_Expansion_range_z", 0.6);
    node_->declare_parameter("octomap.m_isoccupiedThresh", 0.8);
    node_->declare_parameter("octomap.sliding_window", true);
    node_->declare_parameter("octomap.cloud_topic", "/cloud_registered");
    node_->declare_parameter("octomap.map_frame", "world");
    node_->declare_parameter("octomap.sensor_frame", "");
    node_->declare_parameter("octomap.tf_timeout", 0.1);

    node_->get_parameter("octomap.resolution", mp_.resolution);
    node_->get_parameter("octomap.prob_hit", mp_.prob_hit);
    node_->get_parameter("octomap.prob_miss", mp_.prob_miss);
    node_->get_parameter("octomap.occupancy_thresh", mp_.occupancy_thresh);
    node_->get_parameter("octomap.localmap_thresh", mp_.localmap_thresh);
    node_->get_parameter("octomap.max_range", mp_.max_range);
    node_->get_parameter("octomap.pointcloudMaxX", mp_.m_pointcloudMaxX);
    node_->get_parameter("octomap.pointcloudMinX", mp_.m_pointcloudMinX);
    node_->get_parameter("octomap.pointcloudMaxY", mp_.m_pointcloudMaxY);
    node_->get_parameter("octomap.pointcloudMinY", mp_.m_pointcloudMinY);
    node_->get_parameter("octomap.pointcloudMaxZ", mp_.m_pointcloudMaxZ);
    node_->get_parameter("octomap.pointcloudMinZ", mp_.m_pointcloudMinZ);
    node_->get_parameter("octomap.m_compressMap", mp_.m_compressMap);
    node_->get_parameter("octomap.m_Expansion_range_x", mp_.m_Expansion_range_x);
    node_->get_parameter("octomap.m_Expansion_range_y", mp_.m_Expansion_range_y);
    node_->get_parameter("octomap.m_Expansion_range_z", mp_.m_Expansion_range_z);
    node_->get_parameter("octomap.m_isoccupiedThresh", mp_.m_isoccupiedThresh);
    node->get_parameter("octomap.sliding_window", mp_.m_sliding_window);
    node_->get_parameter("octomap.cloud_topic", mp_.cloud_topic);
    node_->get_parameter("octomap.map_frame", mp_.map_frame);
    node_->get_parameter("octomap.sensor_frame", mp_.sensor_frame);
    node_->get_parameter("octomap.tf_timeout", mp_.tf_timeout);

    RCLCPP_INFO(node_->get_logger(), "Octomap parameters loaded: resolution=%.2f\n, prob_hit=%.2f\n, prob_miss=%.2f\n, occupancy_thresh=%.2f\n, localmap_thresh=%.2f\n, max_range=%.2f\n, sliding_window=%d\n",
                mp_.resolution, mp_.prob_hit, mp_.prob_miss, mp_.occupancy_thresh, mp_.localmap_thresh, mp_.max_range, mp_.m_sliding_window);
    RCLCPP_INFO(node_->get_logger(), "Octomap parameters loaded: pointcloudMaxX=%.2f\n, pointcloudMinX=%.2f\n, pointcloudMaxY=%.2f\n, pointcloudMinY=%.2f\n, pointcloudMaxZ=%.2f\n, pointcloudMinZ=%.2f\n",
                mp_.m_pointcloudMaxX, mp_.m_pointcloudMinX, mp_.m_pointcloudMaxY, mp_.m_pointcloudMinY, mp_.m_pointcloudMaxZ, mp_.m_pointcloudMinZ);
    RCLCPP_INFO(node_->get_logger(), "Octomap parameters loaded: m_compressMap=%d\n, m_Expansion_range_x=%.2f\n, m_Expansion_range_y=%.2f\n, m_Expansion_range_z=%.2f\n, m_isoccupiedThresh=%.2f\n",
                mp_.m_compressMap, mp_.m_Expansion_range_x, mp_.m_Expansion_range_y, mp_.m_Expansion_range_z, mp_.m_isoccupiedThresh);
    RCLCPP_INFO(node_->get_logger(), "Octomap TF input: cloud_topic=%s, map_frame=%s, sensor_frame=%s, tf_timeout=%.2f",
                mp_.cloud_topic.c_str(), mp_.map_frame.c_str(), mp_.sensor_frame.c_str(), mp_.tf_timeout);

    // init OctoMap
    m_octree_ = std::make_shared<octomap::OcTree>(mp_.resolution);

    m_octree_->setProbHit(mp_.prob_hit);
    m_octree_->setProbMiss(mp_.prob_miss);
    m_octree_->setOccupancyThres(mp_.occupancy_thresh);

    cloud_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions cloud_sub_opt;
    cloud_sub_opt.callback_group = cloud_callback_group_;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    m_inflated_octree_ = std::make_shared<octomap::OcTree>(*m_octree_);
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        mp_.cloud_topic, rclcpp::SensorDataQoS(), std::bind(&OctomapMapper::cloudCallback, this, std::placeholders::_1),
        cloud_sub_opt);

    point_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);

    rclcpp::QoS qos_profile(10);
    qos_profile.transient_local();  // 
    qos_profile.reliable();
    octomap_pub_ = node_->create_publisher<octomap_msgs::msg::Octomap>("octomap_binary", qos_profile);
    current_time = node_->now();
    RCLCPP_INFO(node_->get_logger(),
                                "octo map initizialized");

}

std::shared_ptr<octomap::OcTree> OctomapMapper::getMap() const{
    return m_octree_;
}

void OctomapMapper::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    const std::string cloud_frame = msg->header.frame_id;
    const std::string sensor_frame = mp_.sensor_frame.empty() ? cloud_frame : mp_.sensor_frame;
    if (cloud_frame.empty() || sensor_frame.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Received point cloud without frame_id and octomap.sensor_frame is empty");
        return;
    }

    geometry_msgs::msg::TransformStamped cloud_to_map;
    try {
        cloud_to_map = tf_buffer_->lookupTransform(
            mp_.map_frame, cloud_frame, msg->header.stamp,
            rclcpp::Duration::from_seconds(mp_.tf_timeout));
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "Failed to transform point cloud from %s to %s: %s",
                    cloud_frame.c_str(), mp_.map_frame.c_str(), ex.what());
        return;
    }

    geometry_msgs::msg::TransformStamped sensor_to_map;
    if (sensor_frame == cloud_frame) {
        sensor_to_map = cloud_to_map;
    } else {
        try {
            sensor_to_map = tf_buffer_->lookupTransform(
                mp_.map_frame, sensor_frame, msg->header.stamp,
                rclcpp::Duration::from_seconds(mp_.tf_timeout));
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(node_->get_logger(), "Failed to transform sensor origin from %s to %s: %s",
                        sensor_frame.c_str(), mp_.map_frame.c_str(), ex.what());
            return;
        }
    }

    sensor_origin_ = octomap::point3d(
        sensor_to_map.transform.translation.x,
        sensor_to_map.transform.translation.y,
        sensor_to_map.transform.translation.z);

    sensor_msgs::msg::PointCloud2 cloud_transformed_msg;
    tf2::doTransform(*msg, cloud_transformed_msg, cloud_to_map);

    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_transformed_msg, *cloud);

    // 2. filter
    auto pc_cloud = filterPointCloud(cloud);
    //publishCloud(pc_cloud);

    // 3. Convert pcl messages to octomap messages
    octomap::Pointcloud octomap_cloud;
    for (const auto& point : pc_cloud->points)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }
        octomap_cloud.push_back(octomap::point3d(point.x, point.y, point.z));
    }
     // Insert point cloud data into OctoMap

    std::lock_guard<std::mutex> lock(octree_mutex_);
     
    m_octree_->insertPointCloud(octomap_cloud, sensor_origin_, mp_.max_range, false, true);
    m_octree_->prune();

    if (mp_.m_sliding_window) {
        clearOldData(sensor_origin_, mp_.localmap_thresh);
    }

    Inflated_octree();

    // Use ray tracing to update octomap
    // insertPointCloud(octomap_cloud);

    publishMap(m_inflated_octree_);
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

}

void OctomapMapper::publishMap(const std::shared_ptr<octomap::OcTree>& octree)
{
    if (!octree || octree->size() == 0)
    {
        RCLCPP_WARN(node_->get_logger(), "Octree is null");
        return;
    }
    
    octomap_msgs::msg::Octomap map;
    map.header.stamp = node_->get_clock()->now();
    map.header.frame_id = mp_.map_frame; 
    const auto& tree_ref = *octree; 
    if (octomap_msgs::binaryMapToMsg(tree_ref, map)) {
        // Publishing OctoMap messages
        // RCLCPP_INFO(node_->get_logger(), "publish octobinaryMap!");
        octomap_pub_->publish(map);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Error serializing OctoMap");
    }
}

void OctomapMapper::publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*input_cloud, output_msg);
    output_msg.header.frame_id = mp_.map_frame;  
    output_msg.header.stamp = node_->get_clock()->now();

   
    point_pub_->publish(output_msg);
}

void OctomapMapper::Inflated_octree() {

    m_inflated_octree_ = std::make_shared<octomap::OcTree>(*m_octree_);
    for (auto it = m_octree_->begin_leafs(); it != m_octree_->end_leafs(); ++it)
    {
        if( m_octree_->isNodeOccupied(*it) ) {
            octomap::point3d occupied_point = it.getCoordinate();
            for ( double dx = -mp_.m_Expansion_range_x; dx <= mp_.m_Expansion_range_x; dx += mp_.resolution )
            {
                for ( double dy = -mp_.m_Expansion_range_y; dy <= mp_.m_Expansion_range_y; dy += mp_.resolution )
                {
                    for ( double dz = -mp_.m_Expansion_range_z; dz <= mp_.m_Expansion_range_z; dz += mp_.resolution )
                    {
                        octomap::point3d inflated_point(occupied_point.x() + dx, 
                                                        occupied_point.y() + dy, 
                                                        occupied_point.z() + dz);
                        m_inflated_octree_->updateNode(inflated_point, true);
                    }
                }
            }
        }
    }

}

double OctomapMapper::getResolution() const{
    return m_octree_->getResolution();
}

bool OctomapMapper::getInflateOccupancy(const Eigen::Vector3d& pos) {
  // 0) sanity check: do we even have an inflated tree?
  if (!m_inflated_octree_) {
    RCLCPP_ERROR(node_->get_logger(),
      "getInflateOccupancy(): inflated octree not set");
    return false;  
  }

  // 1) convert to OctoMap point
  float x = static_cast<float>(pos.x());
  float y = static_cast<float>(pos.y());
  float z = static_cast<float>(pos.z());
  octomap::point3d p{x, y, z};

  // 2) search for the leaf
  auto* node = m_inflated_octree_->search(p);
  if (!node) {
    // no leaf → treat as “free” (or adjust as your logic needs)
    return false;
  }

  // 3) compare against your threshold
  return (node->getOccupancy() >= mp_.m_isoccupiedThresh);
}


void OctomapMapper::clearOldData(const octomap::point3d& center, double radius) {
    octomap::point3d min_bound(
        center.x() - radius,
        center.y() - radius,
        center.z() - radius
    );
    octomap::point3d max_bound(
        center.x() + radius,
        center.y() + radius,
        center.z() + radius
    );

    std::vector<octomap::OcTreeKey> keys_to_delete;

    for (auto it = m_octree_->begin_leafs(); it != m_octree_->end_leafs(); ++it) {
        const octomap::point3d& point = it.getCoordinate();
        if (point.x() < min_bound.x() || point.x() > max_bound.x() ||
            point.y() < min_bound.y() || point.y() > max_bound.y() ||
            point.z() < min_bound.z() || point.z() > max_bound.z()) {
            // add the key to the deletion list
            keys_to_delete.push_back(it.getKey());
        }
    }
    // delete nodes outside the bounding box
    for (const auto& key : keys_to_delete) {
        m_octree_->deleteNode(key, false); // false means do not prune immediately
    }


}
