# Aurora 930 Depth Camera

**Hardware:** Deptrum Aurora 930 (RGBD — color + depth + IR + point cloud)
**Status:** Working on ROS2 Jazzy / Ubuntu 24.04 aarch64

---

## Driver

Built from source — no official Jazzy support exists, Humble tarball patched and compiled.

**Location:** `ros2arm_ws/src/deptrum-ros-driver-aurora930/`
**Original tarball:** `ArmPi_Ultra_Resources/3. Hardware Resources/4. Camera Documentation/1. Aurora Depth Camera/aurora930/2 Configuration and Usage in ROS/3 ROS2 Configuration and Usage/Package/deptrum-ros-driver-humble-aarch64.tar.gz`

### Jazzy patches applied
Two header renames required (`.h` → `.hpp`):
- `cv_bridge/cv_bridge.h` → `cv_bridge/cv_bridge.hpp` (3 files)
- `tf2_geometry_msgs/tf2_geometry_msgs.h` → `tf2_geometry_msgs/tf2_geometry_msgs.hpp` (1 file)
- Added Jazzy detection in `CMakeLists.txt`

### Build (already done)
```bash
sudo apt-get install -y libusb-1.0-0-dev libgl1-mesa-dev libglu1-mesa-dev libxi-dev libudev-dev libgoogle-glog-dev libgflags-dev
cd ~/ros2arm_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select deptrum-ros-driver-aurora930 --cmake-args -DSTREAM_SDK_TYPE=AURORA930
```

### udev rules (run once)
```bash
sudo cp ~/ros2arm_ws/src/deptrum-ros-driver-aurora930/ext/deptrum-stream-aurora900-linux-aarch64-v1.1.19-18.04/scripts/99-deptrum-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## Launch

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2arm_ws/install/setup.bash
ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py
```

---

## Topics

Topics publish under `/aurora/` namespace (native driver, no remapping).

| Topic | Type | Notes |
|---|---|---|
| `/aurora/rgb/image_raw` | `sensor_msgs/Image` | BGR8 color |
| `/aurora/depth/image_raw` | `sensor_msgs/Image` | 16UC1, values in **mm** |
| `/aurora/ir/image_raw` | `sensor_msgs/Image` | Infrared |
| `/aurora/points2` | `sensor_msgs/PointCloud2` | 3D point cloud — TODO: learn |
| `/aurora/rgb/camera_info` | `sensor_msgs/CameraInfo` | Color intrinsics |
| `/aurora/ir/camera_info` | `sensor_msgs/CameraInfo` | Depth intrinsics |

---

## Viewing output

```bash
# Color or depth image
ros2 run rqt_image_view rqt_image_view

# RViz (color + depth + point cloud together)
ros2 run rviz2 rviz2
# Fixed Frame: depth_camera_link
# Add by topic: /aurora/rgb/image_raw, /aurora/depth/image_raw, /aurora/points2
```

---

## Reading frames in a node

```python
import numpy as np
import message_filters
from sensor_msgs.msg import Image, CameraInfo

rgb_sub   = message_filters.Subscriber(self, Image, '/aurora/rgb/image_raw')
depth_sub = message_filters.Subscriber(self, Image, '/aurora/depth/image_raw')
info_sub  = message_filters.Subscriber(self, CameraInfo, '/aurora/ir/camera_info')

sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 2)
sync.registerCallback(self.callback)

def callback(self, rgb_msg, depth_msg, info_msg):
    rgb   = np.ndarray((rgb_msg.height, rgb_msg.width, 3), dtype=np.uint8, buffer=rgb_msg.data)
    depth = np.ndarray((depth_msg.height, depth_msg.width), dtype=np.uint16, buffer=depth_msg.data)
    # depth[y, x] = distance in mm
    # intrinsics: fx=info_msg.k[0], fy=info_msg.k[4], cx=info_msg.k[2], cy=info_msg.k[5]
```

### Pixel → 3D point
```python
z = depth[y, x] / 1000.0  # mm → meters
X = (x - cx) * z / fx
Y = (y - cy) * z / fy
# point = (X, Y, z) in camera frame
```

---

## Reference source examples

`ArmPi_Ultra_Resources/Source Code/ROS2/src/example/example/rgbd_function/include/`

- `get_depth_rgb_img.py` — side-by-side RGB + colorized depth display
- `distance_measure.py` — click a pixel to read depth; auto-finds nearest point
- `rgb_depth_to_pointcloud.py` — Open3D point cloud, ground plane removal via RANSAC
