/*
Exemplary Command Algorithm for the Deflector Robot
Node that publishes time-varying setpoints to a ROS Control node
Author: Roberto Zegers
Date: 2019-May-28
*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>

// global variables
// create variables of type "Float64" to keep the joint position command (in meters)
std_msgs::Float64 j1_pos_msg;
std_msgs::Float64 j2_pos_msg;
std_msgs::Float64 j3_pos_msg;

int switch_branch = 1; // keeps track of the next decision branch on the decision tree

// joint displacement limits
// TO-DO: read from parameters/launch file/xacro
const float j1_lo_limit = 0.0; // prismatic joint 1 lower limit
const float j1_up_limit = 2.8; // upper limit = lenght_l1 - x_width_l2

float remap(float fromValue, float fromMin, float fromMax, float toMin, float toMax) {
  float fromAbs = fromValue - fromMin;
  float fromMaxAbs = fromMax - fromMin;
  float normal = fromAbs / fromMaxAbs;
  float toMaxAbs = toMax - toMin;
  float toAbs = toMaxAbs * normal;
  float toValue = toAbs + toMin;
  return toValue;
}

// Decision tree for determining joint 1 position commands
void decision_step() {
  if (switch_branch == 1) {
    // calculate target joint position
    int j_target = 25; // value between 0 and 100
    j1_pos_msg.data = remap(j_target, 0, 100, j1_lo_limit, j1_up_limit);
    switch_branch = 2; // advance the branch for next decision step
    }
  else if (switch_branch == 2) {
    // calculate target joint position
    int j_target = 50; // value between 0 and 100
    j1_pos_msg.data = remap(j_target, 0, 100, j1_lo_limit, j1_up_limit);
    switch_branch = 3; // advance the branch for next decision step
    }
  else if (switch_branch == 3) {
    // calculate target joint position
    int j_target = 75; // value between 0 and 100
    j1_pos_msg.data = remap(j_target, 0, 100, j1_lo_limit, j1_up_limit);
    switch_branch = 1; // loop back the branch for next decision step
    }
  else {
    ROS_WARN("unknown 'switch_branch' value\n");
  }
} // end of decision_step()

int main(int argc, char ** argv) {
  ros::init(argc, argv, "deflector_commander"); // init ROS and set name of this node
  ros::NodeHandle nh; // Create a ROS NodeHandle object

  ROS_INFO("Demo joint trajectory started!");
  ros::Publisher j1_position_pub = nh.advertise<std_msgs::Float64>("/deflector/joint1_position_controller/command", 1);
  ros::Publisher j2_position_pub = nh.advertise<std_msgs::Float64>("/deflector/joint2_position_controller/command", 1);
  ros::Publisher j3_position_pub = nh.advertise<std_msgs::Float64>("/deflector/joint3_position_controller/command", 1);

  int num_loops = 300; // Will publish a setpoint for num_loops/loopRate

  ros::Rate loopRate(50); // in Hz, in this example loop every 6 seconds
  // run node continuously (which is usefull for this example)
  while (ros::ok()) {
    decision_step(); // execute conditionals to decide on next joint 1 commands

    // move joint 1 to next target position
    for (int i = 0; i < num_loops; i++) // Publish data for num_loops/loopRate seconds
    {
      j1_position_pub.publish(j1_pos_msg); // publish the value
      loopRate.sleep();
    }
    // move joint 2 to an elevated position
    j2_pos_msg.data = -0.7; // set an elevated position
    for (int i = 0; i < num_loops; i++) // Publish data for num_loops/loopRate seconds
    {
      j1_position_pub.publish(j1_pos_msg); // publish the value
      j2_position_pub.publish(j2_pos_msg); // publish the value
      loopRate.sleep();
    }
    // extend pusher (joint 3)
    j3_pos_msg.data = 0.7;
    for (int j = 0; j < num_loops; j++) // Publish data for num_loops/loopRate seconds
    {
      j1_position_pub.publish(j1_pos_msg); // publish the value
      j2_position_pub.publish(j2_pos_msg); // publish the value
      j3_position_pub.publish(j3_pos_msg); // publish the value
      loopRate.sleep();
    }
    // retract pusher (joint 3)
    j3_pos_msg.data = 0.0;
    for (int k = 0; k < num_loops; k++) // Publish data for num_loops/loopRate seconds
    {
      j1_position_pub.publish(j1_pos_msg); // publish the value
      j2_position_pub.publish(j2_pos_msg); // publish the value
      j3_position_pub.publish(j3_pos_msg); // publish the value
      loopRate.sleep();
    }
    // move joint 2 to back to the bottom
    j2_pos_msg.data = 0.0;
    for (int i = 0; i < num_loops; i++) // Publish data for num_loops/loopRate seconds
    {
      j1_position_pub.publish(j1_pos_msg); // publish the value
      j2_position_pub.publish(j2_pos_msg); // publish the value
      loopRate.sleep();
    }

  } // end of while loop

}
