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

# run the commander algorithm
xterm  -e  "rosrun deflector_robot deflector_robot"
