
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

3. **Run the Autonomous Navigation Script:**

```bash
cd autonomous
python3 r2auto_nav.py
```

---

## 🌍 Real-World System Testing (Autonomous Mode)

1. **Connect to Raspberry Pi:**

Try using the alias:

```bash
sshrp
```

If it fails with an error like:

```
ssh: Could not resolve hostname : No address associated with hostname
```

Connect manually instead:

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
python3 elec_nodes.py
```

5. **Activate the Ball Launcher Node:**

```bash
python3 ball_nodes.py
```

6. **Run the Autonomous Navigation Script:**

```bash
cd autonomous
python3 r2auto_nav.py
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