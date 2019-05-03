#!/bin/sh
echo "Deflector Robot Simulation and example command algorithm"
echo "Make sure you have sourced devel/setup.bash"
echo ""
xterm  -e  "roslaunch deflector_robot gazebo.launch" &
sleep 6

xterm  -e  "roslaunch deflector_robot spawn_robot.launch" & 
sleep 6

xterm  -e  "roslaunch deflector_robot ctrl_manager.launch" & 
sleep 6

# run the position commander algorithm
xterm  -e  "roslaunch deflector_robot position_cmd.launch joint_nr:=1"
