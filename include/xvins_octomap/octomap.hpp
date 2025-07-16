#ifndef OCTOMAP_HPP
#define OCTOMAP_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include "nav_msgs/msg/odometry.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

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
   void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
   void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);

   void publishMap();
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
    

private:
    std::shared_ptr<rclcpp::Node> node_;
    // OctoMap 
    std::shared_ptr<octomap::OcTree> m_octree_;
    std::shared_ptr<octomap::OcTree> m_inflated_octree_;

    // ROS sub and pub
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

    // params
    struct Params {
        double resolution;
        double prob_hit;
        double prob_miss;
        double occupancy_thresh;
        double localmap_thresh;

        double m_pointcloudMaxX;
        double m_pointcloudMinX;
        double m_pointcloudMaxY;
        double m_pointcloudMinY;
        double m_pointcloudMaxZ;
        double m_pointcloudMinZ;

        double m_Expansion_range_x;
        double m_Expansion_range_y;
        double m_Expansion_range_z;

        double m_isoccupiedThresh;

        bool m_compressMap;
        bool m_loadMap;
        bool m_sildWindow;

        std::string map_path;
    } mp_;

    bool build_map_xvins;

    Eigen::Matrix3f R;
    
    cv::Mat camera_matrix_;
    Eigen::Matrix4f T_imu_cam_;
    

    octomap::point3d sensor_origin_;
    octomap::KeyRay m_keyRay;
    octomap::KeySet free_cells;
    octomap::KeySet occupied_cells;
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;

   
};

#endif  // OCTOMAP_HPP