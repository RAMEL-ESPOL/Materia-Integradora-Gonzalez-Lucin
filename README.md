# Interactive Posture Game System for Children with ADHD

This project implements a therapeutic system based on ROS 2, computer vision, and a simulated humanoid robot. Its purpose is to support the treatment of children with ADHD through a posture memory game guided by the robot, with emotional tracking, auditory feedback, and a mobile application for therapists.

---

## Overview

The system consists of three main components:

1. A set of ROS 2 nodes responsible for game logic, posture validation, and emotion recognition.  
2. A simulation environment in Gazebo that represents the humanoid robot.  
3. An Android mobile application connected via WebSocket to visualize the child’s results and control the game.  

---

## System Requirements

### Recommended Operating System

- Ubuntu 22.04 LTS (Linux)

### ROS 2

Developed for **ROS 2 Humble**. Follow the [official installation guide](https://docs.ros.org/en/humble/Installation.html).

---

## Step-by-step Installation

### 1. Create the workspace

```bash
mkdir -p ~/ros2tesis_ws/src
cd ~/ros2tesis_ws
```

### 2. Clone the repository

```bash
cd src
git clone https://github.com/RAMEL-ESPOL/Materia-Integradora-Gonzalez-Lucin.git
cd ..
```

### 3. Install system dependencies

**Option A — ROS-GZ (Ignition/modern Gazebo)**

```bash
sudo apt update && sudo apt install -y   python3-colcon-common-extensions python3-pip python3-opencv   ros-humble-desktop   ros-humble-moveit   ros-humble-ros2-control ros-humble-ros2-controllers   ros-humble-ros-gz ros-humble-gz-ros2-control   ros-humble-rosbridge-suite   ros-humble-warehouse-ros-mongo   ros-humble-usb-cam ros-humble-xacro
```

**Option B — Classic Gazebo**

```bash
sudo apt update && sudo apt install -y   python3-colcon-common-extensions python3-pip python3-opencv   ros-humble-desktop   ros-humble-moveit   ros-humble-ros2-control ros-humble-ros2-controllers   ros-humble-gazebo-ros   ros-humble-rosbridge-suite   ros-humble-warehouse-ros-mongo   ros-humble-usb-cam ros-humble-xacro
```

### 4. Install ROS 2 dependencies for the project

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 4.5 Optional — Create a virtual environment for dependencies

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

### 5. Build the project

```bash
colcon build
source install/setup.bash
```

Add to `.bashrc`:

```bash
echo "source ~/ros2tesis_ws/install/setup.bash" >> ~/.bashrc
```

---

## Running the System

### 1. Start the robot simulation

```bash
ros2 launch yaren_gazebo_sim coco_robot.launch.py
```

**Alternative (for low-resource CPU/GPU computers):**

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch yaren_gazebo_sim coco_robot.launch.py
```

### 2. Start the main service

```bash
ros2 run yaren_services launch_master_service.py
```

### 3. Connect the mobile app via WebSocket

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

## Mobile Application

Developed in **Kotlin (Android Studio)**.

1. Open the project in Android Studio.  
2. Connect a physical device or emulator.  
3. Run the project.  

Inside the app:  
- Enter the IP of the computer running ROS 2.  
- Press *Connect*.  
- Start the game or access reports.  

---

## Project Package Structure

| Package | Function |
|---------|----------|
| `posture_game` | Game logic, feedback, statistics, posture validation, robot movement, emotion detection |
| `coco_gazebo_sim` | Gazebo simulation environment to visualize robot movements |
| `coco_services` | Service to send patient names and remotely start launches |
| `rosbridge_server` | WebSocket communication with the mobile app |
| `coco_ws` | Contains robot information such as URDF, xacro, RViz and Gazebo communication, plus additional implementations |
| `dynamixel_sdk` | Communication layer between ROS 2 and ROBOTIS Dynamixel servomotors |

---

## Package Structure: yaren_memory (posture memory game)

| File | Function |
|------|----------|
| `game_manage_node.py` | Node that controls game logic, feedback, statistics, and assistance |
| `detect_emotions_node.py` | Node that processes images and classifies them using a trained machine learning model |
| `mediapipe_node.py` | Node that performs image processing with MediaPipe and publishes raw topics for other nodes |
| `speaker_node.py` | Node that converts text to speech via a TTS system |
| `checker_node.cpp` | Node that validates postures according to response time and required pose |
| `movement_publisher_node.cpp` | Node that publishes robot joint configuration commands to move the robot |
| `postures.cpp` | Script containing conditions to validate a posture based on MediaPipe landmarks |

---

## FAQ

**Can I use the system without the physical robot?**  
Yes. It works entirely in simulation using Gazebo.

**Which camera is recommended?**  
Any USB camera compatible with `usb_cam`.

**Where are the reports stored?**  
As `.json` files, accessible from the mobile app.

**Where can I find the mobile application repository?**  
The repository is available at: [https://github.com/RAMEL-ESPOL/YarenApp.git](https://github.com/RAMEL-ESPOL/YarenApp.git)