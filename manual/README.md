
# 🚀 Quick Start

## 📁 Clone the Repository

```bash
git clone https://github.com/kenpegrasio/r2auto_nav_CDE2310.git
cd r2auto_nav_CDE2310
```

---

## 🧪 Virtual Testing in Gazebo (Simulation)

1. **Launch the Gazebo Simulation:**

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. **Start SLAM with Cartographer:**

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

3. **Enable Manual WASD Control (RTELEOP):**

```bash
rteleop
```

---


## 🎮 Real-World System Testing (Manual Mode)

1. **Connect to Raspberry Pi:**

```bash
sshrp
```

Or connect manually if needed:

```bash
ssh ubuntu@<ip-address>
```

2. **Start Core Components:**

```bash
rosbu
```

3. **Launch R-SLAM (Cartographer):**

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

4. **Activate the Heat Sensor Node:**

```bash
python3 manual_nav.py
```

5. **Activate the Ball Launcher Node:**

```bash
python3 ball_nodes.py
```

6. **Enable Manual WASD Control (RTELEOP):**

```bash
rteleop
```

---

## 🛠 Requirements

- **ROS 2 Humble**
- **TurtleBot3 packages**
- **Cartographer SLAM**
- **Gazebo (for simulation)**

---

## 🤖 Robot Setup Checklist

Ensure your TurtleBot3 is:

- Properly calibrated (motors & sensors)
- Connected to the same network as your PC
- Configured with correct ROS 2 environment variables:
  - `ROS_DOMAIN_ID`
  - `ROS_NAMESPACE`
  - Other necessary parameters