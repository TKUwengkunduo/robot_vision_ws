# Robot Vision Workspace

This repository contains the implementation of a robot vision system using YOLOv8 and ROS 2. The system includes object detection and image display nodes for real-time processing and visualization.

## Prerequisites

- ROS 2 Humble
- Python 3.8+
- Install required Python packages:
  ```bash
  pip install ultralytics opencv-python cv_bridge ament_index_python
  ```

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/TKUwengkunduo/robot_vision_ws.git
   cd robot_vision_ws
   ```
2. Build the workspace
   ```bash
   colcon build
   ```


## Running the YOLOv8 Object Detection Node
   ```bash
   source install/setup.bash
   ros2 run yolov8_obg_det_pkg yolov8_obg_det
   ```
## Running the Display Node
   ```bash
   source install/setup.bash
   ros2 run display_pkg display_node
   ```

Maintainer: Weng Kun Duo
Email: wengkunduo@gmail.com