# Generic Mobile Robot SDF Models

Three Gazebo SDF 1.6 models for ROS 2 Jazzy automation testing.
All models use generic, brand-neutral names and are safe for commercial demo use.

---

## Models at a Glance

```
Research Circular Robot          Mini Educational Robot      Breadboard Hobby Robot
┌──────────────────────────┐     ┌─────────────────┐        ┌──────────────────┐
│  Circular chassis        │     │  Mini circular   │        │  Rectangular box │
│  r = 0.156 m             │     │  r = 0.037 m     │        │  115×92×45 mm    │
│  height ≈ 0.204 m        │     │  h = 0.030 m     │        │                  │
│                          │     │                  │        │  Rear-driven     │
│  Wheel sep: 0.240 m      │     │  Sep: 0.091 m    │        │  Sep: 0.092 m    │
│  Wheel r:   0.052 m      │     │  Wheel r: 0.019  │        │  Wheel r: 0.029  │
│                          │     │                  │        │                  │
│  Sensors:                │     │  Sensors:        │        │  Sensors:        │
│  • 8 sonar (ps0–ps7)     │     │  • 8 IR (ps0–7)  │        │  • 2 IR (rds/lds)│
│  • IMU (accel+gyro)      │     │  • 3 line (ls)   │        │  • 2 whiskers    │
│  • 1 caster (rear)       │     │  • 1 caster(fwd) │        │  • 2 light sens  │
│                          │     │                  │        │  • 2 LEDs        │
│  Mass: 1.600 kg          │     │  Mass: 0.175 kg  │        │  Mass: 0.350 kg  │
└──────────────────────────┘     └─────────────────┘        └──────────────────┘
```

---

## Files

```
robots_worlds/
├── research_circular_robot/
│   ├── model.config
│   └── model.sdf
├── mini_educational_robot/
│   ├── model.config
│   └── model.sdf
└── breadboard_hobby_robot/
    ├── model.config
    └── model.sdf
```

---

## ROS 2 Topic Map

| Topic | Type | Direction |
|---|---|---|
| `/<robot>/cmd_vel` | `geometry_msgs/Twist` | IN |
| `/<robot>/odom` | `nav_msgs/Odometry` | OUT |
| `/<robot>/joint_states` | `sensor_msgs/JointState` | OUT |
| `/research_circular_robot/imu/data` | `sensor_msgs/Imu` | OUT (circular only) |

Where `<robot>` is one of: `research_circular_robot`, `mini_educational_robot`, `breadboard_hobby_robot`.

---

## Spawning in Gazebo (ROS 2 Jazzy)

```bash
source /opt/ros/jazzy/setup.bash

# Research Circular Robot
ros2 run ros_gz_sim create \
  -file ~/Documents/ROSWorkspace/Automation-testing/robots_worlds/research_circular_robot/model.sdf \
  -name research_circular_robot -x 0 -y 0 -z 0.1

# Mini Educational Robot
ros2 run ros_gz_sim create \
  -file ~/Documents/ROSWorkspace/Automation-testing/robots_worlds/mini_educational_robot/model.sdf \
  -name mini_educational_robot -x 0 -y 0 -z 0.05

# Breadboard Hobby Robot
ros2 run ros_gz_sim create \
  -file ~/Documents/ROSWorkspace/Automation-testing/robots_worlds/breadboard_hobby_robot/model.sdf \
  -name breadboard_hobby_robot -x 0 -y 0 -z 0.07
```

---

## Commercial Use

All three SDFs use **generic geometry** (primitive cylinders, boxes, spheres).
No third-party trademarks, brand names, or proprietary assets are included.
Safe for commercial demos.
