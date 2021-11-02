module.exports = {
  apps : [{
    name: "rviz",
    script: "launch_rviz.bash",
    interpreter: "bash"
  },{
    name: "mapping",
    script: "launch_mapping.bash",
    interpreter: "bash"
  },{
    name: "navigation",
    script: "launch_navigation.bash",
    interpreter: "bash"
  },{
    name: "teleop",
    script: "launch_teleop.bash",
    interpreter: "bash"
  }],
};
