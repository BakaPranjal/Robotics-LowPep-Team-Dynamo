# 🐢 Multi-Robot Simulation with SLAM & NAV2 (ROS 2 Humble)

This project sets up a **multi-robot simulation environment** using **ROS 2 Humble**, **Gazebo**, for SLAM, and **Nav2** for navigation. It features **TurtleBot4** robots, **map merging**, and real-time visualization in **RViz2**.

---

## 📦 Installation Instructions

### ✅ 1. Install ROS 2 Humble
Follow the official [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).  
Install the **desktop version** and source the setup script:
```bash
source /opt/ros/humble/setup.bash
```

---

### ✅ 2. Install Gazebo Classic
For simulation support:
```bash
sudo apt update
sudo apt install gazebo
```

---

### ✅ 3. Install RViz2
If not already installed:
```bash
sudo apt install ros-humble-rviz2
```

---

### ✅ 4. Install TurtleBot4 Simulator
```bash
sudo apt install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes
```

---

### ✅ 5. Install Cartographer and Nav2
```bash
sudo apt install ros-humble-cartographer ros-humble-navigation2
```

---

### ✅ 6. Install Dependencies Using `rosdep`
```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 🚀 Running the Simulation

### 📌 1. Launch Multi-Robot Simulation with SLAM & Nav2

**Launch Robot 1**
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py model:=lite namespace:=robot1 slam:=true nav2:=true x:=1.0 y:=0.0 
```

**Launch Robot 2**
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_spawn.launch.py namespace:=robot2 x:=1.5 y:=0.0 model:=lite slam:=true nav2:=true
```

---

### 📌 2. Visualize in RViz2

**View Robot 1**
```bash
ros2 launch turtlebot4_viz view_model.launch.py namespace:=/robot1
```

**View Robot 2**
```bash
ros2 launch turtlebot4_viz view_model.launch.py namespace:=/robot2
```

---

### 📌 3. Merge Maps
Combine maps from multiple robots:
```bash
ros2 launch merge_map merge_map_launch.py
```

---

### 📌 4. Save the Generated Map
Save the explored map with a custom name:
```bash
ros2 run nav2_map_server map_saver_cli -f <map_name>
```
Replace `<map_name>` with your desired filename.

---

## 📖 Notes
- Make sure to source the ROS 2 environment in each terminal session after installing.
- Adjust robot positions (`x` and `y` parameters) based on your simulation layout.

---

## 🤖 Credits
Built with ❤️ using **ROS 2**, **Gazebo**, **Nav2**, and **TurtleBot4** by **Team Dynamo**.
