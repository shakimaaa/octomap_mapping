#ifndef OCTOMAP_HPP
#define OCTOMAP_HPP

#define OCTOMAP_NODEBUGOUT

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/srv/bounding_box_query.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>
#include <unordered_map>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

class OctomapMapper {
public:
   // default constructor
   OctomapMapper() = default;

   // node initialization
   void initMap(std::shared_ptr<rclcpp::Node> node);

   std::shared_ptr<octomap::OcTree> getMap() const;

   /* Callback Functions */
   void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

   void publishMap(const std::shared_ptr<octomap::OcTree>& octree);
   void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

   //  point cloud filter
   pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

   inline void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min);

   inline void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max);

   void insertPointCloud(octomap::Pointcloud &octomap_cloud);

   void Inflated_octree();

   bool getInflateOccupancy(const Eigen::Vector3d& pos);

   double getResolution() const;

   void clearOldData(const octomap::point3d& center, double radius);
   void clearOutsideLocalBox();
   void clearVisibleOccupiedCells(const octomap::Pointcloud& octomap_cloud, double current_time);
   void updateObservedCells(const octomap::Pointcloud& octomap_cloud, double current_time);
    

private:
    std::shared_ptr<rclcpp::Node> node_;
    // OctoMap 
    std::shared_ptr<octomap::OcTree> m_octree_;
    std::shared_ptr<octomap::OcTree> m_inflated_octree_;

    // ROS sub and pub
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_pub_;

    rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;

    std::mutex octree_mutex_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // params
    struct Params {
        double resolution;
        double prob_hit;
        double prob_miss;
        double occupancy_thresh;
        double localmap_thresh;
        double max_range;

        double m_pointcloudMaxX;
        double m_pointcloudMinX;
        double m_pointcloudMaxY;
        double m_pointcloudMinY;
        double m_pointcloudMaxZ;
        double m_pointcloudMinZ;

        double m_localmapMaxX;
        double m_localmapMinX;
        double m_localmapMaxY;
        double m_localmapMinY;
        double m_localmapMaxZ;
        double m_localmapMinZ;

        double m_Expansion_range_x;
        double m_Expansion_range_y;
        double m_Expansion_range_z;

        double m_isoccupiedThresh;

        float m_voxel_size;

        bool m_compressMap;
        bool m_loadMap;
        bool m_sildWindow;
        bool m_sliding_window; // Whether to use sliding window for map management
        bool m_visibility_cleanup;
        double visibility_cleanup_rate;
        double dynamic_clear_duration;

        double tf_timeout;
        std::string cloud_topic;
        std::string map_frame;
        std::string sensor_frame;
        std::string map_path;
    } mp_;

    octomap::point3d sensor_origin_;
    octomap::KeyRay m_keyRay;
    octomap::KeySet free_cells;
    octomap::KeySet occupied_cells;
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;
    std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> occupied_last_seen_;

    rclcpp::Time current_time;
    double last_visibility_cleanup_time_{0.0};
};

#endif  // OCTOMAP_HPP
