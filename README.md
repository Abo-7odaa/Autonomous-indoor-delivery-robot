# Autonomous Indoor Delivery Robot 🤖📦

This project focuses on the development of an Autonomous Mobile Robot (AMR) using **ROS Noetic** on **Ubuntu 20.04**. 
---


## 🛠️ How to use


### 1. Setup Environment
```bash
# Clone the repository
git clone https://github.com/Abo-7odaa/Autonomous-indoor-delivery-robot.git

# Build the workspace
cd Autonomous-indoor-delivery-robot
catkin_make
source devel/setup.bash
 ```
----
### 2. Mapping
#### 1- Using G-Mapping
To launch the robot in Gazebo (Simulation):
```bash
$roslaunch rafiq_bringup mapping.launch
 ```
To lanunch pyhsical robot (using Rpi 4)
```bash
$roslaunch rafiq_bringup mapping_pi.launch
```

#### 2- using Hector SlAM (Real Hardwarw only)
```bash
$roslaunch hector_slam_launch tutorial.launch
```
----
### 3. Navigation
To launch the robot in Gazebo (Simulation):
```bash
$roslaunch rafiq_bringup navigation.launch
```
To lanunch pyhsical robot (using Rpi 4)
```bash
$roslaunch rafiq_bringup navigation_pi.launch
```
---

## ⚠️ Important Note for Real Hardware Deployment

Before using the **Real Hardware launch files**, please ensure the following configurations are met to prevent any hardware damage or navigation errors:

1. **URDF Adaptation:** Modify the `robot_description` (Xacro/URDF) files to precisely match your physical robot's dimensions, mass, and sensor positions.
2. **Encoder Parameters:** Verify critical parameters in your motor controller config, especially the **Ticks Per Revolution (TPR)** and wheel diameter, to ensure accurate Odometry.
3. **On-board Environment:** Ensure that the **Raspberry Pi** (or your SBC) has the proper environment installed:
   * **ROS Noetic** (properly sourced).
   * All required sensor drivers.
   * Communication libraries (like `rosserial`.
4. **Emergency Stop:** It is **highly recommended** to integrate a physical emergency switch into the robot's power circuit.
