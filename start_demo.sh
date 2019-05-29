#!/bin/sh
echo ""
echo "Deflector Robot Simulation and demo command algorithm"
echo ""
xterm  -e  "roslaunch deflector_robot gazebo.launch" &
sleep 6

xterm  -e  "roslaunch deflector_robot spawn_robot.launch" & 
sleep 4

# Spawn table
xterm  -e  "roslaunch deflector_robot spawn_sdf_models.launch" & 
sleep 4

xterm  -e  "roslaunch deflector_robot ctrl_manager.launch" & 
sleep 6

# run the position commander algorithm
# xterm  -e  "roslaunch deflector_robot position_cmd.launch joint_nr:=1"
xterm  -e  "rosrun deflector_robot joint_commander_example"
