# 📘 Research Track 2 – Second Assignment
**Student:** Mohammadhossein Baba  
**ID:** S5919466

📄 **Live Documentation:**  
👉 [View the Sphinx-generated documentation here](https://mohammadhoseinbaba.github.io/ResearchTrack2/)

---

## 🧠 Project Overview

This ROS package is part of the second assignment for the Research Track II course. It interacts with a simulated robot in Gazebo and consists of three main ROS nodes that handle robot navigation, target tracking, and performance analysis.

---

## 🚀 Nodes Overview

### 🅰️ Node A – Goal Management + Kinematics Publisher
- Implements an **action client** for setting and canceling targets.
- Publishes robot's position and velocity on `/pos_vel` using a **custom message**.
- Gets initial goal from parameters `/des_pos_x` and `/des_pos_y`.

### 🅱️ Node B – Last Target Service
- Implements a **service** that returns the last target coordinates set by the user.
- Uses ROS parameters to fetch goal position.

### 🅲 Node C – Velocity & Distance Analyzer
- Subscribes to `/pos_vel`.
- Implements a **service** to calculate:
  - Average velocity (sliding window)
  - Distance to last target
- Window size is set using `/window_size`.

---

## 📂 Package Structure

```
assignment_2_2023/
├── scripts/
│   ├── my_node_A.py
│   ├── my_node_B.py
│   └── my_node_C.py
├── msg/
│   └── Vel.msg
├── srv/
│   ├── Input.srv
│   └── Ave_pos_vel.srv
├── launch/
│   └── assignment1.launch
├── config/
│   └── sim.rviz
├── CMakeLists.txt
└── package.xml
```

---

## 📡 Custom Message

### `Vel.msg`
| Field  | Description             |
|--------|-------------------------|
| pos_x  | Robot's x position      |
| pos_y  | Robot's y position      |
| vel_x  | Linear velocity (x)     |
| vel_z  | Angular velocity (z)    |

---

## 🔁 Custom Services

### `Input.srv`
Returns last goal coordinates:
- `input_x` (float)
- `input_y` (float)

### `Ave_pos_vel.srv`
Returns:
- `distance` (float): Distance from current position to last goal
- `average_vel_x` (float): Average linear velocity along x

---

## ⚙️ Launch File

### `assignment1.launch`
Launches all three nodes and initializes parameters:
- `/des_pos_x` and `/des_pos_y` – initial goal coordinates
- `/window_size` – number of velocity values to average over

---

## 🧪 How to Run

### 1. Make scripts executable:
```bash
chmod +x scripts/*.py
```

### 2. Launch simulation:
```bash
roslaunch assignment_2_2023 assignment1.launch
```

---

## 📞 Calling Services

To call the velocity + distance service and get live data:
```bash
rosservice call /info_service
```

---

## 📄 Sphinx Documentation

This project is fully documented using Sphinx. The documentation includes:
- Detailed module-level and class-level docstrings
- Auto-generated API docs
- Live source code linking

👉 **[Click here to view the live documentation](https://mohammadhoseinbaba.github.io/ResearchTrack2/)**

To build it locally:
```bash
cd source/
make html
```

The output will be placed in `build/html`.

---

## ✅ Final Notes

This assignment showcases:
- Modular ROS node design
- Real-time robot state feedback via custom messages
- Service-based communication
- Clean documentation pipeline with Sphinx + GitHub Pages
