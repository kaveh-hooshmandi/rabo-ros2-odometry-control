# 🤖 Rabo Robot ROS 2 - Odometry & Control

This repository contains the ROS 2 source code for the mobile robot **Smart Rabo**, focusing on **odometry**, **robot localization**, **motion control**, and **sensor fusion** using **Kalman Filter (KF)** and **Extended Kalman Filter (EKF)** techniques.

If you're passionate about self-driving technologies and want to build a real robot capable of **autonomous navigation**, this project is for you.


![Smart Rabo Robot](images/rabo_robot.png)
---

## 📚 Key Concepts Covered

- Sensor Fusion
- Kalman Filter
- Probability Theory
- Robot Kinematics
- Odometry
- Robot Localization
- Control Systems

The software is written primarily in **Python** and designed for **ROS 2 Humble** on **Ubuntu 22.04**.

---

## 🛠 Prerequisites

To set up and run this project, you’ll need:

1. **Ubuntu 22.04** installed (either on a physical PC or virtual machine)  
   👉 Download ISO: [https://ubuntu.com/download/desktop](https://ubuntu.com/download/desktop)

2. **ROS 2 Humble** installed on Ubuntu  
   👉 Install guide: [https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

3. **Required ROS 2 packages:**  
   Install the additional packages with the following command:

```bash
sudo apt-get update && sudo apt-get install -y \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control \
  ros-humble-gazebo-ros2-control \
  ros-humble-joint-state-publisher-gui \
  ros-humble-joy \
  ros-humble-joy-teleop \
  ros-humble-turtlesim \
  ros-humble-robot-localization \
  ros-humble-tf-transformations
```

---

## 🚀 Usage

To launch and simulate the Smart Rabo robot:

### 1. Clone the Repository

Clone the GitHub repository to your local machine:

```bash
git clone https://github.com/kaveh-hooshmandi/rabo-ros2-odometry-control.git
cd rabo-ros2-odometry-control/src
```

### 2. . Install Dependencies & Build the ROS 2 Workspace

Install all necessary dependencies using rosdep. This ensures that all required packages (excluding your own source code) are installed:

```bash
rosdep install --from-paths src -y --ignore-src
```
If this is your first time using rosdep, initialize it first with:
```bash
sudo rosdep init
rosdep update
```
Use `colcon` to build all packages in the workspace:

```bash
colcon build
```

### 3. Source the Workspace

Source the setup file to overlay the workspace:

```bash
source install/setup.bash
```

### 4. Launch Simulation and Visualization

You can launch Gazebo simulation and RViz display with the following commands:

#### Launch Gazebo simulation:
```bash
ros2 launch rabo_description gazebo.launch.py
```

#### Launch Gazebo simulation with specific world (e.g., small house):
```bash
ros2 launch rabo_description gazebo.launch.py world_name:=small_house
```

#### Launch RViz for visualization:
```bash
ros2 launch rabo_description display.launch.py
```

### 5. Control the Robot

#### Launch the robot controller node:
```bash
ros2 launch rabo_controller controller.launch.py
```

#### Use keyboard teleoperation to drive the robot:
```bash
ros2 run rabo_controller keyboard_teleop.py
```

#### Alternatively, publish velocity commands manually:
```bash
ros2 topic pub /rabo/cmd_vel geometry_msgs/msg/TwistStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
twist:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0"
```

### 6. Run the Kalman Filter Localization Node

Launch the Kalman filter for odometry and localization:

```bash
ros2 run rabo_localization kalman_filter.py
```

## 🤝 Contributing

Contributions are what make the open-source community such a great place to learn and grow. Any contributions you make are **greatly appreciated**.

### How to Contribute

1. **Fork the Project**
2. **Create a Feature Branch**  
   ```bash
   git checkout -b feature/AmazingFeature
   ```
3. **Commit Your Changes**  
   ```bash
   git commit -m "Add AmazingFeature"
   ```
4. **Push to Your Branch**  
   ```bash
   git push origin feature/AmazingFeature
   ```
5. **Open a Pull Request**

---

## 📫 Contact

**Kaveh Hooshmandi**  
🔗 LinkedIn: [https://www.linkedin.com/in/kaveh-hooshmandi](https://www.linkedin.com/in/kaveh-hooshmandi)  
🔗 Email: dk.hooshmandi@gmail.com
🔗 Cite: [ https://robohooshmand.ir]( https://robohooshmand.ir) 

---
