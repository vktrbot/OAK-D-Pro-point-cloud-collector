This program uses ROS 2 Humble to run the OAK-D Pro camera driver, starts a collector node that receives data from the camera, and provides a trigger service to save a point cloud as a `.ply` file along with metadata in a `.json` file describing the captured frame.


### Output Visualization (.ply)

The captured point clouds are saved in the standard `.ply` format, which preserves both spatial coordinates (XYZ) and color data (RGB). 

To view and inspect the generated .ply files, I used CloudCompare, a free and open-source 3D point cloud processing software. 

**Examples of captured point clouds viewed in CloudCompare:**

<img width="1479" height="798" alt="image" src="https://github.com/user-attachments/assets/785b2b08-b458-492d-9fba-e2a0f9b36882" />

<img width="1132" height="658" alt="image" src="https://github.com/user-attachments/assets/d9aa070b-ca57-4f97-8f44-bd25c5d99011" />





### Metadata Format (.json)
When a snapshot is triggered, alongside the point cloud file, a `.json` metadata file is generated. This file contains temporal, spatial, and configuration details about the captured frame.

**Example of saved metadata:**

```json
{
  "wall_time_iso": "2026-02-18T15:51:18+0200",
  "ros_stamp_sec": 1771422676,
  "ros_stamp_nanosec": 639906334,
  "frame_id": "oak_rgb_camera_optical_frame",
  "point_count": 169659,
  "bbox_min": [
    -39.10368347167969,
    -23.747339248657227,
    0.6320000290870667
  ],
  "bbox_max": [
    40.12569808959961,
    22.03306770324707,
    65.53500366210938
  ],
  "topic": "/oak/points",
  "out_dir": "/home/jetson/Desktop/OAKD/clouds",
  "max_points": 300000,
  "voxel_size": 0.01,
  "format": "ply",
  "binary": true,
  "has_rgb": true
}
```

### Running the program

**1. Terminal — OAK-D Driver**
```bash
source /opt/ros/humble/setup.bash

ros2 launch depthai_ros_driver camera.launch.py \
  pointcloud.enable:=true \
  enable_depth:=true \
  enable_color:=true \
  rectify_rgb:=true \
  depth_module.depth_profile:="640,400,30" \
  rgb_camera.color_profile:="1280,720,30" ```


**2. Terminal — Collector**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run oak_cloud_collector snapshot --ros-args \
  -p voxel_size:=0.01 \
  -p max_points:=300000 ```


**3. Terminal — Service call / Trigger (save snapshot)**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 service call /save_snapshot std_srvs/srv/Trigger "{}" ```

