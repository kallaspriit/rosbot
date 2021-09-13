# ROS Bot

ROS 2 based robot learning platform.

## Base ros setup

https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html#linux-install-binary-install-missing-dependencies

/opt/ros/foxy

- sudo apt install -y python3-rosdep
- rosdep update
- rosdep install --from-paths /opt/ros/foxy/share --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps osrf_testing_tools_cpp poco_vendor rmw_connextdds rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers"


## Creating workspace

https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html

- mkdir -p ~/dev_ws/src
- cd ~/dev_ws/src
- git clone https://github.com/ros/ros_tutorials.git -b foxy-devel
- rosdep install -i --from-path src --rosdistro foxy -y
- colcon build
- open new terminal to source overlay
- cd ~/dev_ws

## Build package
- create workspace directory `mkdir ros2_ws` etc
- create "src" in this directory
- copy the library/package into src
- `rosdep install -i --from-path src --rosdistro foxy -y` to install dependencies
- cd back to workspace and run `colcon build` to build the package

## Teleop robot controller
- https://github.com/ros-teleop/teleop_tools/tree/foxy-devel
- https://ubuntu.com/blog/the-teleop_tools-arrive-in-ros-2-dashing
- `ros2 run mouse_teleop mouse_teleop`
- `ros2 run mouse_teleop mouse_teleop --ros-args -r holonomic:=true`
- `ros2 topic echo /mouse_vel`

## Joystick teleop
- https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/
- https://github.com/ros2/teleop_twist_joy/tree/foxy/
- http://wiki.ros.org/joy
- `sudo apt install ros-foxy-teleop-twist-joy`
- `ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' joy_dev:='dev/input/js1'`
- `jstest-gtk`
- `ros2 topic echo /joy`
- `ros2 topic echo /cmd_vel`

# run ros/test_odrive_ros2_control package
- https://ros-controls.github.io/control.ros.org/getting_started.html
- https://github.com/ros-controls/ros2_control
- https://github.com/ros-controls/ros2_controllers
- https://github.com/ros-controls/ros2_control_demos
- https://github.com/ros-controls/ros2_controllers/tree/master/diff_drive_controller
- http://wiki.ros.org/urdf
- https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md
- https://github.com/ros-controls/ros2_control_demos/blob/master/ros2_control_demo_bringup/config/diffbot_diff_drive_controller.yaml
- https://github.com/ros-controls/ros2_control_demos/tree/master/ros2_control_demo_description/diffbot_description
- https://github.com/ros-controls/ros2_control_demos/blob/master/ros2_control_demo_bringup/launch/diffbot_system.launch.py
- https://ros-controls.github.io/control.ros.org/ros2_control/ros2controlcli/doc/userdoc.html#ros2controlcli-userdoc CLI
- `cd ~/rosbot/ros/test_odrive_ros2_control/`
- `sudo apt install ros-foxy-ros2-control ros-foxy-ros2-controllers`
- `ros2 launch odrive_bringup odrive.launch.py enable_joint1:=true`
- `ros2 topic pub -r 100 /joint0_velocity_controller/commands std_msgs/Float64MultiArray "data: [1]"`
- `ros2 topic pub -r 100 /joint1_velocity_controller/commands std_msgs/Float64MultiArray "data: [-1]"`

## Run ros/test_odrive_ros2 package
- `cd ~/rosbot/ros/test_odrive_ros2/`
- `ros2 run odrive_ros2 odrive_node`
- `ros2 service call /connect_odrive std_srvs/srv/Trigger`
- `ros2 service call /request_state odrive_interfaces/srv/AxisState "{axis: 0, state: 8}"` (not needed if started in closed loop control)
- `ros2 service call /velocity_cmd odrive_interfaces/srv/VelocityControl "{axis: 0, turns_s: 0.5}"`
- `ros2 topic echo /barrery_percentage` (typo.. and not sure of the format, returns something like 3.5)
- `ros2 topic echo /joint_state`

## Create package

cd ~/dev_ws/src
ros2 pkg create --build-type ament_cmake <package_name>
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
ros2 pkg create --build-type ament_cmake --node-name talker test_chat

## Develop C++

https://github.com/ros2/rclcpp
http://docs.ros2.org/latest/api/rclcpp/
http://wiki.ros.org/std_msgs

## Develop nodejs

https://github.com/RobotWebTools/rclnodejs
https://github.com/RobotWebTools/rclnodejs-cli
http://robotwebtools.org/rclnodejs/docs/0.20.0/index.html
https://github.com/RobotWebTools/rclnodejs/tree/develop/example
npx generate-ros-messages

## Run odrive

ros2 launch odrive_bringup odrive.launch.py enable_joint1:=true