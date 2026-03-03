# late_fusion_for_yolos_cpp

Multi-camera YOLO Detection2D and debug image fusion node for ROS 2 (Jazzy).
This is a add-on part of my previous repository "https://github.com/Pavankumarsp02/ros2_yolos_cpp.git"

This package fuses:
- vision_msgs/msg/Detection2DArray
- sensor_msgs/msg/Image (debug images)

from up to 6 YOLO detector nodes into:

- /fused/detections
- /fused/debug_image (2x3 stitched grid)


------------------------------------------------------------
FEATURES
------------------------------------------------------------

- Supports 1–6 detector nodes
- Fresh-only detection fusion (no reuse across cycles)
- 2x3 image stitching layout
- Fully parameterized via YAML
- Compatible with ros2_yolos_cpp
- Built using ament_cmake
- Designed for ROS 2 Jazzy


------------------------------------------------------------
DEPENDENCIES
------------------------------------------------------------

- ROS 2 Jazzy
- rclcpp
- sensor_msgs
- vision_msgs
- cv_bridge
- image_transport
- OpenCV

Install cv_bridge if needed:

  sudo apt install ros-jazzy-cv-bridge


------------------------------------------------------------
USAGE
------------------------------------------------------------

1) Clone into your ROS 2 workspace:
```
  cd ~/ros2_ws/src
  git clone https://github.com/Pavankumarsp02/late_fusion_for_yolos_cpp.git
```
2) Build the workspace:
```
  cd ~/ros2_ws
  colcon build --package-select late_fusion_for_yolos_cpp
```
3) Source the workspace:
```
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash
```
4) Launch fusion:
```
  ros2 launch late_fusion_for_yolos_cpp launch_fusion_node.py
```
5) Ensure YOLO detectors are active (if using ros2_yolos_cpp lifecycle nodes):
```
  ros2 lifecycle set /yolos_detector1 activate
  ros2 lifecycle set /yolos_detector2 activate
  ros2 lifecycle set /yolos_detector3 activate
  ros2 lifecycle set /yolos_detector4 activate
  ros2 lifecycle set /yolos_detector5 activate
  ros2 lifecycle set /yolos_detector6 activate
```

------------------------------------------------------------
PARAMETERS
------------------------------------------------------------

Defined in config/late_fusion_params.yaml

- Input Detection and Image Topics
- Grid size: Rows and Columns
- Tile Width and Height

------------------------------------------------------------
IMAGE STITCHING LAYOUT
------------------------------------------------------------

Fixed 2x3 grid:
```
  0   1   2
  3   4   5
```
The order of img_inputs determines placement:

- img_inputs[0] → top-left
- img_inputs[1] → top-middle
- img_inputs[2] → top-right
- img_inputs[3] → bottom-left
- img_inputs[4] → bottom-middle
- img_inputs[5] → bottom-right

To change the layout, reorder entries in late_fusion_params.yaml.
No rebuild required (unless not using symlink install).


------------------------------------------------------------
DETECTION FUSION LOGIC
------------------------------------------------------------

- Only detections received since the last publish cycle are used.
- Fresh detections are appended into a single Detection2DArray.
- The fused message uses:
    header.frame_id = "fused"
    header.stamp = now()

This is append-only aggregation, not geometric or semantic fusion.

------------------------------------------------------------
LICENSE
------------------------------------------------------------

Apache-2.0
