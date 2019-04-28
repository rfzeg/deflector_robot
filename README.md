## The deflector_robot Package

A Gazebo simulation of a two degree-of-freedom robot with prismatic joints.

### Description
This robot consists of a linear system that works in the X-Y coordinate space.
Its joint configuration provides the ability to functions as a sliding sorter or pusher. When the lower sliding axis reaches the location of an item (on a table, storage rack or conveyor belt) it can extend its upper arm to grab or push that item to a container bin or aftersort lane.   
Cartesian robots are cost effective, simple to make and easy to control or program.  
  
This implementation includes:    
Robot description using **xacro**, simulation in **Gazebo**, position control with **ROS Control** and
a exemplary node to send target joint positions usign a **C++ ROS Publisher**.

### Requirements

+ Ubuntu 16.04 LTS
+ ROS Kinetic
+ Gazebo 7.0
+ ros_control
+ gazebo_ros_control


### Installation

Clone this repository into a ROS catkin workspace:
```
$ cd catkin_ws/src
$ git clone https://github.com/rfzeg/deflector_robot.git
```

Then build and source the workspace:

```
$ cd catkin_ws
$ catkin_make
$ source devel/setup.bash
```

Run the robot in Gazebo with `./start_demo.sh`   
or alternatively:   
`roslaunch deflector_robot demo.launch`   

### Sanity Checks

On a new terminal run this command to see the existing parameters:

`$ rosparam list`

Check that the parameters defined in the deflector_control.yaml file were loaded into the param server.

Then check that the controller manager is running:
`$ rosservice list | grep controller_manager`

Now when typing `$ rostopic list`, several topic like /deflector/joint2_position_controller/command should display. 

### Activate the robot manually

To control the robot joints in Gazebo, one has to publish a joint value with a message type std_msgs/Float64 to one of the joint position controller command topics.
  
For example:  
`$ rostopic pub /deflector/joint2_position_controller/command std_msgs/Float64 "data: 0.5"`

It is recomended to start typing the command above and press the Tab key until the message auto-completes.  If the auto-complete does not work, source your workspace and try again.

Joint2 takes values ranging from 0.0 to 1.9.  
Joint3 receives values send between 0.0 to 0.9.

### Troubleshooting
If the package build fails due to an "Could not find a package configuration file provided by 'controller_manager'" error message, then the controller_manager package is not installed.  
Use: `$ sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers`
Important, then install:
`$ sudo apt install ros-kinetic-gazebo-ros-control`   
