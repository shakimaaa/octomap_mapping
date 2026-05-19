# octomap_mapping

Chinese documentation: [README.zh-CN.md](README.zh-CN.md)

`octomap_mapping` is a ROS 2 OctoMap mapping node. It subscribes to an input point cloud, transforms the cloud into the map frame with TF, updates occupied and free space with OctoMap probabilistic occupancy mapping, and publishes an inflated binary OctoMap for RViz visualization or upstream planning modules.

## Environment Dependence

### Install octomap from source

[octomap-1.9.7](https://github.com/OctoMap/octomap/releases/tag/v1.9.7)

Build Only octomap (Core Library)

```bash
cd octomap-1.9.7
cd octomap
mkdir build
cd build
cmake ..
make
sudo make install
```

Build Full Package (octomap + octovis)

```bash
cd octomap-1.9.7
mkdir build && cd build
cmake ..
make
```

### Other dependencies

```bash
sudo apt install ros-${ROS_DISTRO}-octomap-msgs
```

### rviz2 octomap visualization

```bash
sudo apt-get install ros-${ROS_DISTRO}-octomap-rviz-plugins
```

from source

```bash
cd ~/work_ws
git clone https://github.com/OctoMap/octomap_rviz_plugins.git -b ros2
colcon build --packages-select octomap_rviz_plugins
source install/setup.bash
rviz2
```

## Build

Build from the ROS 2 workspace root:

```bash
cd ~/work_ws
colcon build --packages-select octomap_mapping
source install/setup.bash
```

If your workspace path is different, replace `~/work_ws` with the actual workspace root.

## Usage

### 1. Prepare input data

The node expects the following inputs:

- Point cloud topic: configured by `octomap.cloud_topic`, for example `/cloud_registered` in `fast_lio.yaml` and `/lio/cloud_world` in `super_lio.yaml`.
- TF: the transform from `octomap.map_frame` to the point cloud `frame_id` must be available.
- Sensor origin TF: `octomap.sensor_frame` is used as the ray-casting origin for OctoMap insertion. If it is empty, the point cloud `frame_id` is used.

If TF is incomplete, the node prints `Failed to transform point cloud` or `Failed to transform sensor origin` and skips the current cloud frame.

### 2. Start the node

Use the Fast-LIO configuration:

```bash
ros2 launch octomap_mapping fast_lio.launch.py
```

Use the Super-LIO configuration:

```bash
ros2 launch octomap_mapping super_lio.launch.py
```

You can also run the node directly with a parameter file:

```bash
ros2 run octomap_mapping octomap_mapping_node --ros-args --params-file src/octomap/octomap_mapping/config/octomap_mapping_v3.yaml
```

### 3. Inspect output

The node publishes these topics:

- `octomap_binary`: `octomap_msgs/msg/Octomap`, binary OctoMap, with frame set to `octomap.map_frame`.
- `filtered_cloud`: `sensor_msgs/msg/PointCloud2`. The publisher exists, but publishing is disabled by default because the `publishCloud(pc_cloud)` call is commented out.

Check topics:

```bash
ros2 topic list | grep octomap
ros2 topic echo /octomap_binary --once
```

In RViz2, add the `OctoMap` display plugin, select `/octomap_binary` as the topic, and set Fixed Frame to `octomap.map_frame`, such as `world` or `camera_init`.

## Parameters

Main parameters are in `config/*.yaml` under the `octomap.*` namespace.

| Parameter | Description |
| --- | --- |
| `octomap.resolution` | OctoMap resolution in meters. Smaller values provide finer maps but use more CPU and memory. |
| `octomap.prob_hit` | Occupancy update parameter for voxels hit by point-cloud points. |
| `octomap.prob_miss` | Occupancy update parameter for free space along rays. |
| `octomap.occupancy_thresh` | Probability threshold used to classify a voxel as occupied. |
| `octomap.max_range` | Maximum ray insertion range for OctoMap point-cloud integration. |
| `octomap.pointcloudMinX/Y/Z`, `octomap.pointcloudMaxX/Y/Z` | Input point-cloud crop bounds around the sensor origin. Only points inside this range are inserted. |
| `octomap.localmapMinX/Y/Z`, `octomap.localmapMaxX/Y/Z` | Local map retention bounds for the sliding window. |
| `octomap.m_Expansion_range_x/y/z` | Obstacle inflation range applied to occupied voxels before publishing. |
| `octomap.m_isoccupiedThresh` | Occupancy threshold used by `getInflateOccupancy()` when querying the inflated map. |
| `octomap.sliding_window` | Enables local-map sliding-window cleanup. Voxels outside the local bounds are deleted. |
| `octomap.visibility_cleanup` | Enables dynamic obstacle cleanup in visible space. |
| `octomap.visibility_cleanup_rate` | Dynamic cleanup rate in Hz. |
| `octomap.dynamic_clear_duration` | Occupied voxels that are visible but not observed again for this duration are removed. |
| `octomap.cloud_topic` | Input point-cloud topic. |
| `octomap.map_frame` | Output map frame and target frame for transformed point clouds. |
| `octomap.sensor_frame` | Sensor frame used to determine the ray origin. If empty, the point cloud frame is used. |
| `octomap.tf_timeout` | TF lookup timeout in seconds. |

## Mapping Principle

The main pipeline is implemented in `OctomapMapper::cloudCallback()`:

1. Receive a `sensor_msgs/msg/PointCloud2` message.
2. Transform the point cloud from its original `frame_id` into `octomap.map_frame` with TF.
3. Resolve `octomap.sensor_frame` in the map frame and use it as the OctoMap ray insertion origin.
4. Crop the point cloud with PassThrough filters using `pointcloudMin/MaxX/Y/Z` to remove irrelevant points and far noise.
5. Convert the PCL cloud to `octomap::Pointcloud`.
6. Call `octomap::OcTree::insertPointCloud()` to update the map:
   - Voxels hit by points increase occupancy probability with `prob_hit`.
   - Free space along rays from the sensor origin to hit points is updated with `prob_miss`.
   - Voxels above `occupancy_thresh` are treated as occupied.
7. Call `prune()` to merge consistent child nodes and reduce map size.
8. If `sliding_window` is true, delete voxels outside the local window around the sensor.
9. Copy the map and inflate occupied voxels by `m_Expansion_range_x/y/z`.
10. Serialize the inflated map and publish it on `octomap_binary`.

### Local Map and Sliding Window

The sliding window is implemented by `clearOutsideLocalBox()`. The window bounds are configured by `localmapMin/MaxX/Y/Z` and move with `sensor_origin_`. This prevents the global map from growing without bound and is better suited for local obstacle avoidance, local planning, and simulation tests.

### Obstacle Inflation

`Inflated_octree()` first copies the raw `m_octree_`, then iterates over all occupied leaf nodes. Around each occupied voxel, it marks neighboring voxels as occupied within `m_Expansion_range_x/y/z` using `resolution` as the step size. The node publishes `m_inflated_octree_`, so RViz and planning modules receive an obstacle map with safety margin.

### Dynamic Cleanup

When `visibility_cleanup` is enabled, the node records the last time each occupied cell was hit by the point cloud. If a later point-cloud ray passes through an occupied cell and that cell has not been observed again for more than `dynamic_clear_duration`, the cell is deleted. This is intended to clear dynamic obstacles that have moved away from visible space.

## Notes

- The `filtered_cloud` publisher is created, but `publishCloud(pc_cloud)` is commented out in `cloudCallback()`, so filtered clouds are not published by default.
- `octomap.localmap_thresh` is a legacy local-map radius parameter. Current sliding-window cleanup mainly uses `localmapMin/MaxX/Y/Z`.
- `octomap.m_compressMap` is read from parameters, but the current code always calls `prune()` and publishes a binary OctoMap.
