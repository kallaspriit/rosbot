# odrive_ros2
ROS2 node for ODrive (Firmware version >= v0.5.1)

## Installation Instructions

### Install ODrive tools
[ODrive Docs](https://docs.odriverobotics.com/)

### Install from source
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Get node
git clone https://github.com/skymaze/odrive_ros2.git

# Get interface
git clone https://github.com/skymaze/odrive_interfaces.git

# Build
cd ~/ros2_ws
colcon build
```

## Usage Instructions

### Start odrive node
```bash
source ~/ros2_ws/install/local_setup.bash
# To launch with "ros2 run"
ros2 run odrive_ros2 odrive_node
```

### Parameters

- connection.timeout
- battery.max_voltage
- battery.min_voltage
- battery.topic
- joint_state.topic

### Topics

- barrery_percentage
- joint_state

### Services

- connect_odrive
- request_state
- position_cmd
- velocity_cmd