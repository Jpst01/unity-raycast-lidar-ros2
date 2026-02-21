# Raycast-Based LiDAR Simulation in Unity with ROS2 (Jazzy)

## Overview
This project implements a 2D LiDAR simulation using Unity’s Raycast system and publishes the scan data to ROS2 (Jazzy) for visualization in RViz.

A custom ROS2 bridge (`unity_lidar_bridge`) is developed to receive LiDAR data from Unity over TCP and convert it into `sensor_msgs/msg/LaserScan`.

---

## Features
- 360° LiDAR simulation using Unity Raycasts  
- Real-time distance measurement from virtual environment  
- Custom TCP-based bridge between Unity and ROS2  
- ROS2 LaserScan publishing on `/scan` topic  
- Visualization in RViz  
- Static TF setup (`map → lidar_frame`)  

---

## System Architecture

Unity (Raycast LiDAR)  
→ TCP Socket (JSON data)  
→ ROS2 Bridge (`unity_lidar_bridge`)  
→ `/scan` (LaserScan)  
→ RViz Visualization  

---

## Project Structure

### Unity Project
- `Assets/` – Scripts and scene  
- `Packages/` – Unity dependencies  
- `ProjectSettings/` – Project configuration  

### ROS2 Package: `unity_lidar_bridge`
- `lidar_bridge_node.py` – TCP server + LaserScan publisher  
- `lidar_bridge_launch.py` – Launch file (bridge + static TF)  
- `package.xml`, `setup.py`, `setup.cfg` – ROS2 package setup  

---

## Requirements

### Software
- Unity
- ROS2 Jazzy
- Python 3
- RViz2

## How to Run

### 1. Start ROS2 Bridge (Terminal 1)

Open a terminal and run:

```bash
cd unity-raycast-lidar-ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch unity_lidar_bridge lidar_bridge_launch.py
```

### 2. Run Unity Simulation
- Open project in Unity 
- Press Play
- The LiDAR data will start streaming to ROS2

#### Note
If you dont see any objects in the project then you may need to go to File &rarr; Open Scene &rarr; Select file SampleScene.unity at 
```
~/unity-raycast-lidar-ros2/Assets/Scenes/SampleScene.unity
```

### 3. Visualize in RViz2 (Terminal 2)
```bash
source /opt/ros/jazzy/setup.bash
rviz2
```