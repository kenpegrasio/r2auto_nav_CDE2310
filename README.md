# r2auto_nav_CDE2310

Autonomous navigation system using ROS2 and R-SLAM on TurtleBot3. This repository contains setup instructions for testing both in simulation (Gazebo) and the real world using Turtlebot3.

## 🚀 Quick Start

### Add this repository to your local environment 

1. **Clone this repository**
   ```bash
   git clone https://github.com/kenpegrasio/r2auto_nav_CDE2310.git
   ```

2. **Go to the folder**
   ```bash
   cd r2auto_nav_CDE2310
   ```

### 🧪 Navigation Testing in Gazebo (Virtual Environment)

1. **Launch Gazebo Simulation:**

   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **Launch R-SLAM (Cartographer) in Simulation:**

   ```bash
   ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
   ```

3. **Run Autonomous Navigation Script:**

   ```bash
   cd autonomous
   python3 r2auto_nav.py
   ```

---

### 🌍 System Testing in the Real World

1. **Connect to the Raspberry Pi:**

   Try connecting with the alias:

   ```bash
   sshrp
   ```

   > ⚠️ If this returns an error like:  
   > `ssh: Could not resolve hostname : No address associated with hostname`  
   > Connect manually instead:

   ```bash
   ssh ubuntu@<ip-address>
   ```

2. **Activate Components (on Raspberry Pi terminal):**

   ```bash
   rosbu
   ```

3. **Launch R-SLAM (Cartographer) on the Robot:**

   ```bash
   ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
   ```

4. **Activate the heat sensor node (on Raspberry Pi terminal)**
   ```bash
   python3 elec_nodes.py
   ```

5. **Activate the launching ball node (on Raspberry Pi terminal)**
   ```bash
   python3 ball_nodes.py
   ```

6. **Run Autonomous Navigation Script:**

   ```bash
   cd autonomous
   python3 r2auto_nav.py
   ```

---

## 🛠 Requirements

- ROS 2 (Humble)
- TurtleBot3 packages installed
- Cartographer for SLAM
- Gazebo for simulation

---

## 🤖 Robot Setup

Make sure your TurtleBot3 is properly set up with:
- Calibrated motors and sensors
- Network connection with your PC
- Proper ROS 2 environment variables (`ROS_DOMAIN_ID`, `ROS_NAMESPACE`, etc.)
