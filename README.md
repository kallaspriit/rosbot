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
- create directory
- create "src" in this directory
- copy the library/package into src
- `rosdep install -i --from-path src --rosdistro foxy -y` to install dependencies
- `colcon build` to build the package

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