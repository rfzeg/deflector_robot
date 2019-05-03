## Deflector Robot

A Gazebo simulation of a three degrees-of-freedom robot with prismatic joints.

### Description
This ROS package contains a set of parametrized macros that allow to quickly build custom models of cartesian robots for simulation. The main structural components are three linear actuators that work in the X-Y-Z coordinate space. Cartesian robots are cost effective, simple to make and easy to control or program.
  
With its default robot configuration this package provides a Gazebo model that can function as a sliding sorter or pusher. By positioning its sliding axes in front of an item's location (on a table, storage rack or conveyor belt) it can grab or push that item to a container bin or aftersort lane.     
  
This implementation includes:    
Robot description using **xacro**, debugging robot model in **Gazebo**, position control with **ROS Control**, a exemplary node to send target joint positions usign a **C++ ROS Publisher** and the tools need for **calibrating PID coefficients**.

### Features
+ Parametric Modeling: each dimention is configurable and integrated to a mathematical model of the robot. Unlimited different robot configurations are possible by changing individual variables.
+ Structural elements can be modelled using square or rectangular cross sections.
+ Pre-setup GUI to visualize the controller's performance.
+ Includes ready-to-launch ROS node to show-case the robot movements in simulation.

### Requirements

+ Ubuntu 16.04 LTS
+ ROS Kinetic
+ Gazebo 7.0
+ ros_control
+ gazebo_ros_control

Note: This package has only been tested on Ubuntu 16.04 LTS with ROS Kinetic and Gazebo 7.0.  

### Installation

Clone this repository into a ROS catkin workspace:
```sh
$ cd catkin_ws/src
$ git clone https://github.com/rfzeg/deflector_robot.git
```

Then build and source the workspace, for instance:

```sh
$ cd catkin_ws
$ catkin_make
$ source devel/setup.bash
```

To run a demo launching each node in a separate terminal window: `./start_demo.sh`   
Or alternatively to execute all nodes in one terminal: `roslaunch deflector_robot demo.launch`  

### Optional Checks

On a new terminal run this command to see the existing parameters:

`$ rosparam list`

And check that the parameters defined in the deflector_control.yaml file were loaded into the param server.


Then check that the controller manager is running:
`$ rosservice list | grep controller_manager`

Next, when typing `$ rostopic list`, confirm that several topic like `/deflector/joint1_position_controller/command` display. 

Finally, the robot joints can be verified in Rviz by running:

`roslaunch deflector_robot rviz.launch jsp_gui:=true`  

The following window should appear:

![](doc/imgs/joint_state_publisher.png)  

The sliders of the joint_state_publisher GUI tool can be used to confirm in Rviz that the movement of each joint is correct.  

### Activate the robot manually

To control the robot joints in Gazebo, a joint value with a message of type **std_msgs/Float64** has to has to be published to one of the joint position controller command topics.
  
For example:  
`$ rostopic pub /deflector/joint1_position_controller/command std_msgs/Float64 "data: 0.5"`  

Where data represents the goal position of the joint in meters.

Note: It is recomended to start typing the command above and press the Tab key until the message auto-completes. If the auto-complete does not work, source your workspace and try again.


The allowed joint values for the default robot model in this package are:
+ Joint1 takes values ranging from 0.0 to 2.8 
+ Joint2 accepts values inside limits -0.8 to 0.0  
+ Joint3 receives values between 0.0 to 0.8

### Visualize and tune the PID's controller performance

The PID controller is supplied configured. However if the weights and physical properties of the links are modified, it will be neccesary to re-calibrate the PID gains to get a good performance.

Tune each axis independently. For generating alternating position signals for one axis at a time run:  
`roslaunch deflector_robot position_cmd.launch joint_nr:=1`

To visualize and evaluate the control loop performance run:
`roslaunch deflector_robot rqt.launch`  

Try to reduce the error between:  
`/deflector/joint1_position_controller/command/data`  
and  
`/deflector/joint1_position_controller/state/process_ value`  

The panel **dynamic reconfigure** should be used to tune the proportional, derivative and integral coefficients. While it is very difficult to get to a perfect tune, it is not too difficult to achieve a tune that is good enough for most use cases.

One technique for tuning a PID loop is:
1. Set all gains to zero
2. Adjust Kp as high as you can without inducing wild oscillation (steady oscillations are ok)
3. Increase Kd to remove overshoot
4. Finally adjust Ki to remove any residual offset after the loop has settled 

### Troubleshooting
If the package build fails due to an "Could not find a package configuration file provided by 'controller_manager'" error message, then the controller_manager package is not installed.  
Use:  
`$ sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers`
and then install:  
`$ sudo apt install ros-kinetic-gazebo-ros-control`  

### Resources
+ http://gazebosim.org/tutorials/?tut=ros_control
+ http://gazebosim.org/tutorials?tut=gravity_compensation
+ http://wiki.ros.org/roscpp/Overview/Parameter%20Server
