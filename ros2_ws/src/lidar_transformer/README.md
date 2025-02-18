# Running lidar_transformer

This node takes the lidar point clouds and transforms them to all be in the reference frame of the top lidar (frame id of `os_lidar_top`).

### All 5 LiDARs
It subscribes to `/ouster_back/points`, `/ouster_front/points`, `/ouster_left/points`, `/ouster_right/points`, and `/ouster_top/points`.

It performs the transformations and publishes to `/transformed_lidar/ouster_back/points`, `/transformed_lidar/ouster_front/points`, `/transformed_lidar/ouster_left/points`, `/transformed_lidar/ouster_right/points`, and
`/transformed_lidar/ouster_top/points` all under the fixed frame of `os_lidar_top`.

### Only 1 LiDAR

In the param file (found at `ros2_ws/src/lidar_transformer/config/params.yaml`), comment out all of the topics (or at least the ones you don't want to use) under `lidar_topics: ` except for `/ouster_top/points`. You shouldn't need to change anything else in the param file. 

### Build and run

Build in the `ros2_ws` directory and source the setup file.

```
colcon build --symlink-install && source install/setup.bash
```

> Note: There is a depreciation warning when building this package. It should work just fine if you ignore it.

Launch the node using the following:

```
ros2 launch lidar_transformer lidar_transformer.launch.py viz:=true # viz is optional 
```
> Note: You may need to run `xhost +local:docker` in your host terminal (on uvuntu 24 it works at least) to allow rviz to be launched from within the container.

Then play the bag file for all 5 LiDARs (Named *all_lidars_test1*. Downloadable at link in *ros2_ws/bags/link_to_bags.txt*)

```
ros2 bag play <path_to_bag>
```
