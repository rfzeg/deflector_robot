/*
Exemplary Command Algorithm for the Deflector Robot
Node that publishes time-varying setpoints to a ROS Control node
Author: Roberto Zegers
Date: 2019-May-02
*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>

// global variables
int j1_target; // joint 1 target position in percentage (0 to 100)
double j1_cmd = 0.0; // joint 1 position command in meters
int switch_branch = 1; // keeps track of the next decision branch on the decision tree
// joint displacement limits
const double j1_lo_limit = 0.0; // prismatic joint 1 lower limit
const double j1_up_limit = 2.8; // upper limit = lenght_l1 - x_width_l2 

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
    // command joint 1 movement
    j1_target = 25; // set value between 0 and 100
    j1_cmd = remap(j1_target, 0, 100, j1_lo_limit, j1_up_limit);
    switch_branch = 2;
  } else if (switch_branch == 2) {
    // command joint 1 movement
    j1_target = 50; // set value between 0 and 100
    j1_cmd = remap(j1_target, 0, 100, j1_lo_limit, j1_up_limit);
    switch_branch = 3;
  } else if (switch_branch == 3) {
    // command joint 1 movement
    j1_target = 75; // set value between 0 and 100
    j1_cmd = remap(j1_target, 0, 100, j1_lo_limit, j1_up_limit);
    switch_branch = 1;
  } else {
    ROS_WARN("case 'switch_branch' not known\n");
  }
} // end of decision_step()

int main(int argc, char ** argv) {
  ros::init(argc, argv, "deflector_commander"); // init ROS and set name of this node
  ros::NodeHandle n; // Create a ROS NodeHandle object
  ros::Publisher pos_cmd_joint1 = n.advertise < std_msgs::Float64 > ("/deflector/joint1_position_controller/command", 1);
  //ros::Publisher pos_cmd_3 = n.advertise<std_msgs::Float64>("/deflector/joint2_position_controller/command", 1); 

  //// Recover private parameter from the launch file
  // declare variable to get value from parameter server
  int joint_number;
  // Use a default value of 99 in case the parameter doesn’t exist
  ros::param::param<int>("~joint", joint_number, 99);
  ROS_DEBUG("Got param: %d", joint_number);

  std_msgs::Float64 pos_cmd_float64; //create a variable of type "Float64"
  int num_loops = 300; // Will publish a setpoint for num_loops/loopRate

  ROS_INFO("Started deflector_commander node...");

  ros::Rate loopRate(50); // in Hz, in this example loop every 6 seconds
  // run node continuously (which is usefull for this example)
  while (ros::ok()) {
    decision_step(); // execute conditionals to decide on next joint commands
    ROS_INFO("Sending target position to joint 1: %.2f", j1_cmd);
    pos_cmd_float64.data = j1_cmd;

    for (int i = 0; i < num_loops; i++) // Publish data for num_loops/loopRate seconds
    {
      pos_cmd_joint1.publish(pos_cmd_float64); // publish the value--of type Float64-- 
      loopRate.sleep();
    }
  } // end of while loop

}
