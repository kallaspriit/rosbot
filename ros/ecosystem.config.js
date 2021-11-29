module.exports = {
  apps : [{
    name: "rviz",
    script: "launch_rviz.bash",
    interpreter: "bash"
  },{
    name: "localization",
    script: "launch_localization.bash",
    interpreter: "bash"
  },{
    name: "navigation",
    script: "launch_navigation.bash",
    interpreter: "bash"
  },{
    name: "slam",
    script: "launch_slam.bash",
    interpreter: "bash"
  },{
    name: "teleop",
    script: "launch_teleop.bash",
    interpreter: "bash"
  }, {
    name: "robot",
    script: "launch_robot.bash",
    interpreter: "bash"
  }, {
    name: "gazebo",
    script: "launch_gazebo.bash",
    interpreter: "bash"
  }],
};
