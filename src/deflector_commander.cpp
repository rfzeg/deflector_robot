/*
Exemplary Command Algorithm for the Deflector Robot
Node that publishes time-varying setpoints to a ROS Control node
Author: Roberto Zegers
Date: 2019-May-03
*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>

// global variables
int j_target; // joint target position in percentage (0 to 100)
float j_cmd = 0.0; // joint position command in meters
int switch_branch = 1; // keeps track of the next decision branch on the decision tree
int joint_number; // to keep value from parameter server

// joint displacement limits
// TO-DO: read from parameters/launch file/xacro
const float j1_lo_limit = 0.0; // prismatic joint 1 lower limit
const float j1_up_limit = 2.8; // upper limit = lenght_l1 - x_width_l2
const float j2_lo_limit = -0.8; // prismatic joint 2 lower limit
const float j2_up_limit = 0.0; // prismatic joint 2 upper limit
const float j3_lo_limit = 0.0; // prismatic joint 3 lower limit
const float j3_up_limit = 0.8; // prismatic joint 3 upper limit

float remap(float fromValue, float fromMin, float fromMax, float toMin, float toMax) {
  float fromAbs = fromValue - fromMin;
  float fromMaxAbs = fromMax - fromMin;
  float normal = fromAbs / fromMaxAbs;
  float toMaxAbs = toMax - toMin;
  float toAbs = toMaxAbs * normal;
  float toValue = toAbs + toMin;
  return toValue;
}

// Decision tree for determining joint position commands
void decision_step() {
  if (switch_branch == 1) {
    // command joint movement
    j_target = 25; // set value between 0 and 100
    if (joint_number == 1) {
      j_cmd = remap(j_target, 0, 100, j1_lo_limit, j1_up_limit);
    } else if (joint_number == 2) {
      j_cmd = remap(j_target, 0, 100, j2_lo_limit, j2_up_limit);
    } else if (joint_number == 3) {
      j_cmd = remap(j_target, 0, 100, j3_lo_limit, j3_up_limit);
    }
    switch_branch = 2;
  } else if (switch_branch == 2) {
    // command joint movement
    j_target = 50; // set value between 0 and 100
    if (joint_number == 1) {
      j_cmd = remap(j_target, 0, 100, j1_lo_limit, j1_up_limit);
    } else if (joint_number == 2) {
      j_cmd = remap(j_target, 0, 100, j2_lo_limit, j2_up_limit);
    } else if (joint_number == 3) {
      j_cmd = remap(j_target, 0, 100, j3_lo_limit, j3_up_limit);
    }
    switch_branch = 3;
  } else if (switch_branch == 3) {
    // command joint movement
    j_target = 75; // set value between 0 and 100
    if (joint_number == 1) {
      j_cmd = remap(j_target, 0, 100, j1_lo_limit, j1_up_limit);
    } else if (joint_number == 2) {
      j_cmd = remap(j_target, 0, 100, j2_lo_limit, j2_up_limit);
    } else if (joint_number == 3) {
      j_cmd = remap(j_target, 0, 100, j3_lo_limit, j3_up_limit);
    }
    switch_branch = 1;
  } else {
    ROS_WARN("case 'switch_branch' not known\n");
  }
} // end of decision_step()

int main(int argc, char ** argv) {
  ros::init(argc, argv, "deflector_commander"); // init ROS and set name of this node
  ros::NodeHandle n; // Create a ROS NodeHandle object
  std::string pub_topic; // variable to keep name of topic to which to publish to

  // Recover private parameter from the launch file
  // Use a default value of 1 in case the parameter doesn’t exist
  ros::param::param < int > ("~joint", joint_number, 99);
  // ROS_DEBUG("Got param: %d", joint_number);

  if (joint_number == 1) {
    pub_topic = "/deflector/joint1_position_controller/command";
  } else if (joint_number == 2) {
    pub_topic = "/deflector/joint2_position_controller/command";
  } else if (joint_number == 3) {
    pub_topic = "/deflector/joint3_position_controller/command";
  } else if (joint_number == 99) {
    pub_topic = "/deflector/joint1_position_controller/command";
    ROS_WARN("Using a default joint value of 1. Got param: %d", joint_number);
    joint_number = 1;
  } else {
    ROS_ERROR("Invalid joint value. Could fall back to default value of 1");
    ros::shutdown();
  }

  ros::Publisher pos_cmd_pub = n.advertise < std_msgs::Float64 > (pub_topic, 1);

  std_msgs::Float64 pos_cmd_float64; //create a variable of type "Float64"
  int num_loops = 300; // Will publish a setpoint for num_loops/loopRate

  ros::Rate loopRate(50); // in Hz, in this example loop every 6 seconds
  // run node continuously (which is usefull for this example)
  while (ros::ok()) {
    decision_step(); // execute conditionals to decide on next joint commands
    ROS_INFO("Target position of joint %d: %.2f", joint_number, j_cmd);
    pos_cmd_float64.data = j_cmd;

    for (int i = 0; i < num_loops; i++) // Publish data for num_loops/loopRate seconds
    {
      pos_cmd_pub.publish(pos_cmd_float64); // publish the value
      loopRate.sleep();
    }
  } // end of while loop

}
