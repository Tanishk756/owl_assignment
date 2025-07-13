#  Orangewood Robotics Simulation Assignment

## 📌 Objective

This project demonstrates a simulated robotic system in Gazebo for the Orangewood assignment. The system includes:
- A collaborative **EC66 robotic arm**
- A **Robotiq 2F-85 parallel-jaw gripper**
- A **wrist-mounted RGB-D camera**

### 🔧 Key Features

- ✅ Full **MoveIt2** configuration for motion planning
- ✅ **Obstacle avoidance** using RViz and joint planning
- ✅ Wrist-mounted **camera integration**
- ✅ RGB-D data capture and point cloud generation
- ✅ Integration of **GraspNet baseline** for 6-DOF grasp pose prediction
- ✅ Visualization of predicted grasp poses and point cloud
- 🚧 Grasp execution and benchmarking (skipped due to plugin issues)

---

## 📁 Folder Structure

```
ros2_ws/
├── src/
│   ├── graspnet-baseline/         # GraspNet baseline cloned repository
│   ├── owl_description/           # Robot URDF/XACROs, Gazebo and camera setup
│   ├── owl_moveit_config/         # MoveIt2 config package
├── install/, build/, log/         # ROS 2 workspace build artifacts
├── README.md                      # Readme 
```

---

## 🚀 Setup Instructions

### 1. Prerequisites

- Ubuntu 22.04 + ROS 2 Humble
- Gazebo
- Python 3.10+
- `colcon`, `xacro`, `moveit`, `gazebo_ros_pkgs`

```bash
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions ros-humble-moveit ros-humble-gazebo-ros2-control
```

---

### 2. Build the Workspace

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

### 3. Launch the Simulation in Gazebo

```bash
ros2 launch owl_description gazebo.launch.py
```

---

## 🎯 GraspNet Integration

### 📸 Dummy Input Setup

RGB, depth, and intrinsics files are located in `graspnet_sim/inputs/`:

- `rgb.png`
- `depth.png`
- `camera_intrinsics.txt`

### 🤖 Predict Grasp Poses

```bash
cd graspnet-baseline
python tools/inference.py 
```

### 📈 Visualize Poses

```bash
cd graspnet_sim/scripts/
python visualize_grasps.py
```

This script loads:
- `grasp_candidates.csv`
- `point_cloud.npy`

And shows top-K grasp poses using `matplotlib`.

---

## 📦 Core & Third-Party Packages Used

### 🔨 Core ROS 2 Packages:
- `ros2_control`, `gazebo_ros2_control`, `controller_manager`
- `robot_state_publisher`, `joint_state_publisher`, `xacro`, `moveit2`

### 🤖 Robot & Planning:
- [MoveIt2](https://moveit.ros.org/)
- [URDF / XACRO](https://wiki.ros.org/xacro)

### 🧠 Grasp Prediction:
- [GraspNet Baseline](https://github.com/graspnet/graspnet-baseline) (MIT License)

---

## 📌 Notes
- GraspNet prediction and visualization are functional via dummy inputs.

---

## 🧑‍💻 Author

Tanishk Singhal  
tanishksinghal6285@gmail.com

---

## 🧾 License

MIT License - see the `LICENSE` file for details.
