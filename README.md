This program uses ROS 2 Humble to run the OAK-D Pro camera driver, starts a collector node that receives data from the camera, and provides a trigger service to save a point cloud as a `.ply` file along with metadata in a `.json` file describing the captured frame.

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
  rgb_camera.color_profile:="1280,720,30"


**2. Terminal — Collector**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run oak_cloud_collector snapshot --ros-args \
  -p voxel_size:=0.01 \
  -p max_points:=300000


**3. Terminal — Service call / Trigger (save snapshot)**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 service call /save_snapshot std_srvs/srv/Trigger "{}"

