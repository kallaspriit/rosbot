# Client-service test

- `rosdep install -i --from-path src --rosdistro foxy -y`
- `colcon build`
- `. install/local_setup.bash`
- `ros2 run client_service sum_server`
- `ros2 run client_service sum_client 2 3`