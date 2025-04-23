# 🧭 Line Follower Robot Simulation (ROS 2 + Ignition Gazebo)

This project simulates a line-following robot using **ROS 2** and **Ignition Gazebo**. The robot detects and follows a black line on the ground using a stereo camera and avoids obstacles using lidar. It features a state machine for reactive behavior and uses OpenCV for computer vision processing.

---

## 🛠️ System Design

The system is implemented as a modular **ROS 2-based architecture** to simulate a line-following robot in **Ignition Gazebo**. It includes:

- A **robot model** defined in an **SDF file**, equipped with a **stereo camera** and a **lidar sensor**.
- A **ROS 2 control node** that processes sensor data to decide the robot's movement.
- A **state machine** for behavior switching between line-following, obstacle detection, avoidance, and line reacquisition.

The main loop involves subscribing to camera and lidar data, processing it, and publishing velocity commands to drive the robot.

---

## 🧠 Algorithm / Model Building

The control algorithm combines **computer vision** with **reactive state-based logic**:

- **Line Detection**:  
  Uses OpenCV (grayscale, blur, thresholding, contour detection) to find a black line.

- **Position Error Calculation**:  
  Computes the line's offset from the image center to adjust the robot's path.

- **PID Control**:  
  Applies a PID controller to calculate the angular velocity needed for smooth turning.

- **Obstacle Detection**:  
  Uses lidar data to detect nearby obstacles in front of the robot.

- **Obstacle Avoidance**:  
  The robot performs timed maneuvers to go around obstacles and reacquire the line.

---

## 🧩 Implementation Steps

1. **Define the Robot Model**  
   - Designed the robot in an SDF file with a stereo camera and lidar.

2. **Build the Control Node**  
   - Subscribed to image and lidar topics.  
   - Processed line detection with OpenCV and calculated steering using PID.  
   - Analyzed lidar data for obstacle detection and avoidance direction.

3. **State Machine Logic**  
   - Implemented robot state transitions (e.g., FOLLOWING_LINE → OBSTACLE_DETECTED → TURNING_AWAY).

4. **Tune Control Parameters**  
   - Adjusted PID gains and robot speeds for smooth line tracking and safe avoidance.

5. **Test in Simulation**  
   - Ran tests in Ignition Gazebo using straight and curved tracks with obstacles.

6. **Debug & Optimize**  
   - Enabled debug visualizations (real-time line detection overlays using OpenCV).

---

## 🧰 Software Tools

- **ROS 2 (Foxy/Humble/etc.)** – Robot control framework.
- **Ignition Gazebo 6** – 3D simulation tool for robotics testing.
- **OpenCV** – For image processing and line detection.
- **NumPy** – For lidar data and numerical processing.
- **cv_bridge** – Converts ROS image messages to OpenCV.
- **rclpy** – ROS 2 client library for Python.
- **Python 3** – Used for all control scripts.

---

## 💻 Build Instructions

First, navigate to your ROS 2 workspace:

```bash
cd ~/line_follower_ws
colcon build --packages-select line_follower_pkg
Source the workspace:

source install/setup.bash

▶️ Run Instructions
1. Launch Ignition Gazebo

ign gazebo line_follower.sdf

This opens the simulated environment with the robot model.
2. Start the Line Follower Node

Open a new terminal, then run:

source ~/line_follower_ws/install/setup.bash
ros2 run line_follower_pkg line_follower

3. Bridge Ignition Topics to ROS 2

In a third terminal:

source ~/line_follower_ws/install/setup.bash
ros2 run ros_ign_bridge parameter_bridge \
/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan \
/world/line_following_world/model/line_follower_robot/link/chassis/sensor/stereo_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image \
/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist

This connects the Gazebo simulation to ROS 2 topics.
🔍 Debug Mode

Real-time debug visuals are enabled:

    Line detection overlay

    Error values

    State display (FOLLOWING_LINE, OBSTACLE_DETECTED, etc.)

    Obstacle presence indication

You’ll see these in a live OpenCV window.
✅ Tested With

    Ubuntu 22.04

    ROS 2 Humble

    Ignition Gazebo 6

    Python 3.10

```
