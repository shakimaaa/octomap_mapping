#ifndef OCTOMAP_HPP
#define OCTOMAP_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/srv/bounding_box_query.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>

class OctomapMapper {
public:
   // 默认构造函数
   OctomapMapper() = default;

   // 显式初始化（传入 ROS 2 节点）
   void initMap(std::shared_ptr<rclcpp::Node> node);

   std::shared_ptr<octomap::OcTree> getMap() const;

   /* Callback Functions */
   void cloudCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg);
   void poseCallback(const nav_msgs::msg::Path::SharedPtr msg);

   void publishMap();

   //  point cloud filter
   pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

   inline void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min);

   inline void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max);

   void insertPointCloud(octomap::Pointcloud &octomap_cloud);
    

private:
    std::shared_ptr<rclcpp::Node> node_;
    // OctoMap 
    std::shared_ptr<octomap::OcTree> m_octree_;

    // ROS sub and pub
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pose_sub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;

    // params
    struct Params {
        double resolution;
        double prob_hit;
        double prob_miss;
        double occupancy_thresh;

        double m_pointcloudMaxX;
        double m_pointcloudMinX;
        double m_pointcloudMaxY;
        double m_pointcloudMinY;
        double m_pointcloudMaxZ;
        double m_pointcloudMinZ;

        bool m_compressMap;
    } mp_;

    

    octomap::point3d sensor_origin_;
    octomap::KeyRay m_keyRay;
    octomap::KeySet free_cells;
    octomap::KeySet occupied_cells;
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;

   
};

#endif  // OCTOMAP_HPP