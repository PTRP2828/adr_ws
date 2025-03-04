# ADR_WS - Autonomous Delivery Robot Workspace

## Introduction
This repository contains the setup and usage instructions for `adr_ws`, a ROS 2 workspace for an autonomous delivery robot.

## Prerequisites
- Ubuntu 24.04
- ROS 2 Jazzy installed
- Docker (if running ROS Noetic in a container)
- Required dependencies:
  ```bash
  sudo apt update && sudo apt install -y python3-colcon-common-extensions python3-rosdep git
  ```

## Setting Up the Workspace

1. Clone the repository:
   ```bash
   mkdir -p ~/adr_ws/src
   cd ~/adr_ws/src
   git clone https://github.com/PTRP2828/adr_ws
   cd ~/adr_ws
   ```

2. Install dependencies:
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:
   ```bash
   colcon build --symlink-install
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```
   To make it permanent, add this line to `~/.bashrc`:
   ```bash
   echo "source ~/adr_ws/install/setup.bash" >> ~/.bashrc
   ```

## Running the Robot

### 1. Bringup the Robot 
Use this command in robot Workspace:
```bash
ssh <piusername>@<pi_ip_address>
cd <your_robot_workspace>
## for adr_ws use
ros2 launch my_robot_launch bringup_launch.py
```

### 2. Statr the  the Robot 
Use this in host pc for running rviz2 and control the robot:
```bash
source ~/adr_ws/install/setup.bash
ros2 launch host_control bringup_launch.py
```




## License
This project is licensed under the MIT License.

