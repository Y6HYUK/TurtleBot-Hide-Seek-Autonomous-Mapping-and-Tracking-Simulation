# My TurtleBot Project

This project demonstrates an end-to-end workflow using **ROS2**, **Gazebo**, **RViz**, and **TurtleBot3** to autonomously map a simulated environment, save the generated map, and (in a later phase) launch multi-robot navigation using the saved map. A custom GUI is provided for starting mapping, stopping the exploration, and saving the map.

## Demo
[Click to watch the video](https://www.youtube.com/watch?v=43lTSowCPSk)

## Overview

1. **Simulation Launch**  
   A ROS2 launch file starts Gazebo and RViz while spawning a TurtleBot3 in a virtual environment.

2. **Mapping and Map Saving**  
   A custom GUI enables the TurtleBot to autonomously explore the environment and build an occupancy grid map. Once the exploration is complete, the map is saved (generating `map.yaml` and `map.pgm`) in the project’s `maps` folder.

3. **Multi-Robot Navigation (Future Work)**  
   In the next phase, a new launch file will use the saved map to spawn multiple TurtleBots simultaneously, enabling them to localize and navigate within the environment using Nav2.

## Features

- **Autonomous Exploration**:  
  The TurtleBot explores the environment autonomously and updates the map in real time.

- **Custom GUI**:  
  The GUI provides simple buttons to start mapping, complete the exploration, and save the final map.

- **Map Saving**:  
  The map is saved using ROS2’s map saver utility (`nav2_map_server`) into `map.yaml` and `map.pgm`, which are stored in the project’s `maps` folder.

- **Planned Multi-Robot Navigation**:  
  A future launch file will load the saved map and spawn multiple TurtleBots for autonomous navigation.

## Prerequisites

- Ubuntu 22.04  
- ROS2 Humble  
- TurtleBot3 packages (including `turtlebot3_gazebo`, `turtlebot3_description`, etc.)  
- Gazebo simulation  
- Nav2 (Navigation2) and slam_toolbox  
- Python 3 and necessary Python libraries (PyQt5, OpenCV, NumPy, PyYAML)

## Installation

1. **Clone the Repository**

   ```bash
   cd ~/turtlebot_ws/src
   git clone https://github.com/yourusername/my_turtlebot_project.git
   ```

2. **Install Dependencies**

   Make sure all required ROS2 packages and Python dependencies are installed. For Python libraries, you can use pip:

   ```bash
   pip3 install pyqt5 opencv-python numpy pyyaml
   ```

3. **Build the Workspace**

   ```bash
   cd ~/turtlebot_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

## Usage

### 1. Launch the Simulation

Run the provided launch file to start Gazebo, RViz, and spawn the initial TurtleBot:

```bash
ros2 launch my_turtlebot_project turtlebot_simulation.launch.py
```

### 2. Run the Mapping GUI

In a new terminal (with ROS2 sourced), start the mapping GUI:

```bash
ros2 run my_turtlebot_project mapping_gui
```

- **Start Mapping**: Click this button to initiate autonomous exploration.
- **Complete Mapping**: Use this to stop the exploration when you think the map is complete.
- **Save Map**: This will call the map saver utility and store `map.yaml` and `map.pgm` in the `maps` folder.

### 3. Future Multi-Robot Navigation

After the map is saved, you can close the current simulation and run a new launch file (to be developed) that:
- Loads the saved map.
- Launches RViz with the map.
- Spawns two (or more) TurtleBots that localize using the saved map and navigate autonomously.

## Troubleshooting

- **Map Not Saving/Loading**:  
  Ensure that the map is saved to the correct directory:  
  `/home/yjh/junhyuk_project/turtlebot_ws/src/my_turtlebot_project/maps`  
  The code uses a relative path conversion (similar to your mapping_gui save_map function) to handle build versus source directory differences.

- **TurtleBot Not Moving**:  
  If the robot receives a goal but does not move, check that Nav2 is properly configured and that the goal topic (e.g., `/move_base_simple/goal`) matches what Nav2 subscribes to.

- **Nav2 and SLAM**:  
  Verify that Nav2 bringup, global/local planners, and costmap nodes are running. This is crucial for navigation to work correctly with the saved map.

## Future Improvements

- **Multi-Robot Navigation**:  
  Develop a new launch file that spawns multiple TurtleBots and integrates Nav2 for coordinated navigation.
- **Advanced Object Detection & Tracking**:  
  Integrate object detection (e.g., YOLOv8) to allow robots to track and follow objects in the environment.
- **GUI Enhancements**:  
  Improve the GUI to provide real-time feedback on mapping progress, robot status, and navigation goals.
