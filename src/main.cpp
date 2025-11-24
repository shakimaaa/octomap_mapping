#include <rclcpp/rclcpp.hpp>
#include "xvins_octomap/octomap.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("xvins_octomap_node");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    // 创建 OctoMap 实例并显式初始化
    OctomapMapper mapper;
    mapper.initMap(node);  // 手动调用初始化

    executor.spin();
    rclcpp::shutdown();
    return 0;
}