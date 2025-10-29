# FDPO-RobotSystem — Complete Robot System

Complete software and firmware repository for the **FDPO team** robot, including both low-level firmware (Raspberry Pi Pico W) and high-level control software (Raspberry Pi 4 with ROS Noetic).

> **Context:** developed by **Electrical and Computer Engineering students at FEUP (Faculty of Engineering, University of Porto)** for the **Robot Factory 4.0** competition at the **National Robotics Festival 2026**.

---

## Repository Structure

This repository contains two main components as Git submodules:

```
FDPO4.0/
├── fdpo-firmware/          # Raspberry Pi Pico W firmware (submodule)
│   └── README.md           # Firmware documentation
├── fdpo-ros-stack/         # Raspberry Pi 4 ROS stack (submodule)
│   └── README.md           # ROS stack documentation
└── README.md               # This file
```

**Architecture:**

* **`fdpo-firmware`** — Low-level firmware running on **Raspberry Pi Pico W** (RP2040)

  * Real-time motor control (25 Hz)
  * Sensor reading (encoders, ToF)
  * Actuator management (motors, electromagnet)
  * Communication interface with Pi4
* **`fdpo-ros-stack`** — High-level software running on **Raspberry Pi 4**

  * ROS Noetic packages
  * Navigation and path planning
  * Localization (LiDAR-based beacon detection + EKF)
  * Driver for Pico W communication

---

## System Overview

### Communication Flow

**Data flow:**

1. **Pi4 sends:** `CMD:<v_ref>,<w_ref>,<pick>` (up to 50 Hz)
2. **Pico W responds:** `POS:<x>,<y>,<theta>, TOF:<obstacle>` (up to 50 Hz)

---

## Quick Start

### 1. Clone the Repository

```bash
# Clone with submodules
git clone --recursive https://github.com/your-org/FDPO4.0.git
cd FDPO4.0

# Or if already cloned, initialize submodules
git submodule update --init --recursive
```

### 2. Update Submodules

```bash
# Update all submodules to latest
git submodule update --remote

# Or update individually
cd fdpo-firmware
git pull origin main
cd ../fdpo-ros-stack
git pull origin main
cd ..
```

### 3. Setup ROS Stack (Pi4)

See [`fdpo-ros-stack/README.md`](fdpo-ros-stack/README.md) for details:

```bash
cd fdpo-ros-stack
catkin_make
source devel/setup.bash
roslaunch conf wake_up_fdpo.launch
```

---

## Requirements

### For Pico W Firmware

* **Hardware:**

  * Raspberry Pi Pico W
  * Pico4Drive motor controller shield
  * Encoders, ToF sensor, solenoids
* **Software:**

  * PlatformIO
  * Arduino framework (Earle Philhower core)

### For ROS Stack

* **Hardware:**

  * Raspberry Pi 4
  * HLDS HLS-LFCD2 LiDAR
  * USB connection to Pico W
* **Software:**

  * Ubuntu 20.04 LTS
  * ROS Noetic
  * Required ROS packages (see `fdpo-ros-stack/README.md`)

---

## Development Workflow

### Working with Submodules

```bash
# Make changes in a submodule
cd fdpo-firmware
# ... make changes ...
git add .
git commit -m "Update firmware"
git push origin main

# Update parent repo to point to new submodule commit
cd ..
git add fdpo-firmware
git commit -m "Update firmware submodule"
```

### Building Everything

```bash
# Build firmware
cd fdpo-firmware && pio run && cd ..

# Build ROS stack
cd fdpo-ros-stack && catkin_make && cd ..
```

---

## System Integration

### Communication Protocol

The system uses a **text-based serial protocol** over USB (115200 baud):

**Commands (Pi4 → Pico W):**

```
CMD:<v_ref>,<w_ref>,<pick>\n
```

**Responses (Pico W → Pi4):**

```
POS:<x>,<y>,<theta>,TOF:<obstacle>\n
```

The ROS driver node (`fdpo-ros-stack`) publishes received odometry as `/odom` and obstacle detection as `/box_detection`.

### Control Architecture

* **Pi4 (High-level):**

  * Path planning → Navigation controller → Velocity commands (`v_ref`, `w_ref`)
  * Localization → Pose correction
  * Task coordination
* **Pico W (Low-level):**

  * Velocity commands → Kinematics → PID → Motor voltages → PWM
  * Encoder feedback → Odometry
  * Sensor reading → Obstacle detection

---

## Troubleshooting

### Submodule Issues

**Submodule out of sync:**

```bash
git submodule update --init --recursive
```

**Reset submodule to tracked commit:**

```bash
git submodule update --remote fdpo-firmware
```

### Communication Issues

* Verify USB connection between Pi4 and Pico W
* Check baud rate (115200)
* Monitor serial on Pi4: `rostopic echo /odom`
* Monitor serial on Pico: `pio device monitor` (115200 baud)

---

## Documentation

* **[Firmware Documentation](fdpo-firmware/README.md)** — Pico W firmware details

  * Dual-core architecture
  * Thread safety and mutexes
  * PID control equations
  * Robot kinematics
* **[ROS Stack Documentation](fdpo-ros-stack/README.md)** — Pi4 software details

  * ROS package structure
  * Navigation controller
  * Localization system
  * LiDAR integration

---

## License

Not defined yet — confirm before external use.

---

## Credits

Developed by **Electrical and Computer Engineering students at FEUP** for **Robot Factory 4.0** (National Robotics Festival 2026):

* *Afonso Mateus*
* *Christian Geyer*
* *Daniel Silva*
* *Pedro Lopes*

---

## References

* [Firmware README](fdpo-firmware/README.md)
* [ROS Stack README](fdpo-ros-stack/README.md)
* [Git Submodules Documentation](https://git-scm.com/book/en/v2/Git-Tools-Submodules)
