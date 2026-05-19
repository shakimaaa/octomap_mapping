# octomap_mapping

English documentation: [README.md](README.md)

`octomap_mapping` 是一个 ROS 2 OctoMap 建图节点。节点订阅输入点云，通过 TF 将点云转换到地图坐标系，使用 OctoMap 概率栅格更新占据/空闲空间，并发布膨胀后的二进制 OctoMap，供 RViz 显示或上层规划模块查询。

## 环境依赖

### 从源码安装 OctoMap

[octomap-1.9.7](https://github.com/OctoMap/octomap/releases/tag/v1.9.7)

只编译 octomap 核心库：

```bash
cd octomap-1.9.7
cd octomap
mkdir build
cd build
cmake ..
make
sudo make install
```

编译完整包 octomap + octovis：

```bash
cd octomap-1.9.7
mkdir build && cd build
cmake ..
make
```

### 其他依赖

```bash
sudo apt install ros-${ROS_DISTRO}-octomap-msgs
```

### RViz2 OctoMap 可视化

```bash
sudo apt-get install ros-${ROS_DISTRO}-octomap-rviz-plugins
```

从源码安装：

```bash
cd ~/work_ws
git clone https://github.com/OctoMap/octomap_rviz_plugins.git -b ros2
colcon build --packages-select octomap_rviz_plugins
source install/setup.bash
rviz2
```

## 编译

在 ROS 2 工作空间根目录编译：

```bash
cd ~/work_ws
colcon build --packages-select octomap_mapping
source install/setup.bash
```

如果你的工作空间路径不同，请把 `~/work_ws` 换成实际的 workspace 根目录。

## 使用方法

### 1. 准备输入数据

节点默认需要以下输入：

- 点云话题：由参数 `octomap.cloud_topic` 指定，例如 `fast_lio.yaml` 中为 `/cloud_registered`，`super_lio.yaml` 中为 `/lio/cloud_world`。
- TF：必须能查询到 `octomap.map_frame` 到点云 `frame_id` 的变换。
- 传感器原点 TF：`octomap.sensor_frame` 用作 OctoMap ray casting 的起点；如果该参数为空，则使用点云自身的 `frame_id`。

如果 TF 不完整，节点会打印 `Failed to transform point cloud` 或 `Failed to transform sensor origin`，并跳过当前帧点云。

### 2. 启动节点

使用 Fast-LIO 配置：

```bash
ros2 launch octomap_mapping fast_lio.launch.py
```

使用 Super-LIO 配置：

```bash
ros2 launch octomap_mapping super_lio.launch.py
```

也可以直接运行节点并指定参数文件：

```bash
ros2 run octomap_mapping octomap_mapping_node --ros-args --params-file src/octomap/octomap_mapping/config/octomap_mapping_v3.yaml
```

### 3. 查看输出

节点发布以下话题：

- `octomap_binary`：`octomap_msgs/msg/Octomap`，二进制 OctoMap，frame 为 `octomap.map_frame`。
- `filtered_cloud`：`sensor_msgs/msg/PointCloud2`，过滤后的点云发布接口已保留，但当前代码中默认没有调用发布。

检查话题：

```bash
ros2 topic list | grep octomap
ros2 topic echo /octomap_binary --once
```

在 RViz2 中添加 `OctoMap` 显示插件，Topic 选择 `/octomap_binary`，Fixed Frame 设置为参数 `octomap.map_frame`，例如 `world` 或 `camera_init`。

## 参数说明

主要参数位于 `config/*.yaml`，参数命名空间为 `octomap.*`。

| 参数 | 作用 |
| --- | --- |
| `octomap.resolution` | OctoMap 分辨率，单位 m。值越小地图越精细，但计算量和内存更大。 |
| `octomap.prob_hit` | 点云命中体素时，提高占据概率的更新参数。 |
| `octomap.prob_miss` | 射线穿过空闲空间时，降低占据概率的更新参数。 |
| `octomap.occupancy_thresh` | 判断体素为 occupied 的概率阈值。 |
| `octomap.max_range` | OctoMap 插入点云时的最大射线范围。 |
| `octomap.pointcloudMinX/Y/Z`、`octomap.pointcloudMaxX/Y/Z` | 以传感器原点为中心的输入点云裁剪范围。只有范围内点云会参与建图。 |
| `octomap.localmapMinX/Y/Z`、`octomap.localmapMaxX/Y/Z` | 滑动窗口局部地图保留范围。 |
| `octomap.m_Expansion_range_x/y/z` | 发布前对 occupied 体素做障碍物膨胀的范围。 |
| `octomap.m_isoccupiedThresh` | `getInflateOccupancy()` 查询膨胀地图时使用的占据阈值。 |
| `octomap.sliding_window` | 是否启用局部地图滑动窗口，启用后删除窗口外体素。 |
| `octomap.visibility_cleanup` | 是否启用可视范围内动态障碍清理。 |
| `octomap.visibility_cleanup_rate` | 动态清理频率，单位 Hz。 |
| `octomap.dynamic_clear_duration` | occupied 体素超过该时间未被再次观测到，并且位于当前可视射线上时会被清理。 |
| `octomap.cloud_topic` | 输入点云话题。 |
| `octomap.map_frame` | 输出地图坐标系，也是点云转换后的目标坐标系。 |
| `octomap.sensor_frame` | 传感器坐标系，用于确定射线起点；为空时使用点云 frame。 |
| `octomap.tf_timeout` | TF 查询超时时间，单位 s。 |

## 建图原理

本节点的核心流程在 `OctomapMapper::cloudCallback()` 中完成：

1. 接收 `sensor_msgs/msg/PointCloud2` 点云。
2. 使用 TF 将点云从原始 `frame_id` 转换到 `octomap.map_frame`。
3. 根据 `octomap.sensor_frame` 获取传感器在地图坐标系下的位置，作为 OctoMap 射线插入的原点。
4. 使用 PassThrough 滤波器按 `pointcloudMin/MaxX/Y/Z` 裁剪点云，减少无关点和远处噪声。
5. 将 PCL 点云转换为 `octomap::Pointcloud`。
6. 调用 `octomap::OcTree::insertPointCloud()` 更新地图：
   - 点云命中的体素按 `prob_hit` 增加占据概率。
   - 传感器原点到命中点之间的射线空间按 `prob_miss` 更新为空闲。
   - 体素概率超过 `occupancy_thresh` 后被认为是 occupied。
7. 对 OctoMap 执行 `prune()`，合并一致的子节点，降低地图规模。
8. 如果 `sliding_window` 为 true，删除传感器局部窗口外的体素，只保留机器人附近局部地图。
9. 根据 `m_Expansion_range_x/y/z` 复制一份膨胀后的 OctoMap，把每个 occupied 体素周围一定范围标记为 occupied。
10. 将膨胀后的地图序列化为 `octomap_binary` 发布。

### 局部地图与滑动窗口

滑动窗口通过 `clearOutsideLocalBox()` 实现。窗口边界由 `localmapMin/MaxX/Y/Z` 决定，并随 `sensor_origin_` 移动。这样可以避免全局地图持续增大，更适合局部避障、局部规划和仿真测试。

### 障碍物膨胀

`Inflated_octree()` 会先复制原始 `m_octree_`，然后遍历所有 occupied 叶子节点，并在 x/y/z 三个方向按 `resolution` 步长扩展到 `m_Expansion_range_x/y/z` 范围内。发布的是 `m_inflated_octree_`，因此 RViz 或规划模块看到的是带安全距离的障碍物地图。

### 动态障碍清理

当 `visibility_cleanup` 开启时，节点会记录 occupied cell 最近一次被点云命中的时间。后续点云射线经过某个 occupied cell，但该 cell 超过 `dynamic_clear_duration` 没有再次被观测命中时，会从地图中删除。这个逻辑用于清理可视区域内已经移动走的动态障碍物。

## 注意事项

- `filtered_cloud` 发布器已创建，但当前 `cloudCallback()` 中 `publishCloud(pc_cloud)` 被注释，默认不会发布滤波后的点云。
- `octomap.localmap_thresh` 是旧的局部地图半径参数，当前滑动窗口清理主要使用 `localmapMin/MaxX/Y/Z`。
- `octomap.m_compressMap` 参数会被读取，但当前代码始终调用 `prune()` 并发布二进制 OctoMap。
