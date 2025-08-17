# ROS2 Cone Detection and Behavior Tree-Based Autonomous Navigation

## Overview

This project enables a ROS2-based robot to detect cones in real time using YOLOv8 and respond autonomously using a custom Behavior Tree (BT) node. When a cone is detected by the `yolov8_ros` detector, it publishes to a ROS topic. The BT node listens to this topic and triggers a 360º rotation maneuver.

Navigation is handled with ROS2 Nav2, and costmap filters dynamically reduce the robot's speed near cones, promoting safer and more precise movements.

The architecture cleanly separates perception (YOLOv8 detection) from behavior (BT-based control), enabling modular and extensible robot autonomy.

## Features

- Real-time cone detection using YOLOv8 within ROS2.
- Custom Behavior Tree node triggers rotation on cone detection.
- Autonomous navigation via ROS2 Nav2.
- Adaptive speed control near cones using costmap filters.
- Modular design separating detection and behavior.

## Project Structure

- `your_custom_bt_node/` - Custom BT node implementing rotation action on cone detection.
- `yolov8_ros/` - ROS2 package for YOLOv8 detection.
- `nav2_params.yaml` - Navigation and costmap filter configuration.
- Behavior Tree XML defining node interactions.

## Usage

### Environment Setup

```bash
export LD_LIBRARY_PATH=/home/trsa2024/trsa_lab3/install/your_custom_bt_node/lib:$LD_LIBRARY_PATH
```


### Launch Commands

1. Start YOLOv8 detection node:
    
```bash
ros2 launch yolov8_bringup yolov8.launch.py
```

2. Launch TurtleBot3 simulation and Nav2 navigation:
 
```bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False world:=/home/trsa2024/trsa_lab3/trsa_tb3_cone.world params_file:=/home/trsa2024/trsa_lab3/nav2_params.yaml
```

3. Start costmap filter info node for dynamic speed control:

```bash
ros2 launch nav2_costmap_filters_demo costmap_filter_info.launch.py
```

## Behavior Tree Highlights

- Listens for cone detections published on a designated topic.
- Triggers a 360° robot rotation upon detection.
- Integrates with Nav2 navigation pipeline.

## Improvements and Future Work

- Improve detection accuracy with more robust training data.
- Add richer behaviors to the Behavior Tree for complex tasks.

## Author

David Furtado

## License

Please refer to individual package license files for usage rights.
