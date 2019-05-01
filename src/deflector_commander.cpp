/*
Exemplary Command Algorithm for the Deflector Robot
Author: Roberto Zegers
Date: 2019-May-01
*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>

float remap(float fromValue, float fromMin, float fromMax, float toMin, float toMax) {
  float fromAbs = fromValue - fromMin;
  float fromMaxAbs = fromMax - fromMin;
  float normal = fromAbs / fromMaxAbs;
  float toMaxAbs = toMax - toMin;
  float toAbs = toMaxAbs * normal;
  float toValue = toAbs + toMin;
  return toValue;
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "deflector_commander"); // init ROS and set name of this node
  ros::NodeHandle n; // Create a ROS NodeHandle object
  ros::Publisher pos_cmd_joint1 = n.advertise < std_msgs::Float64 > ("/deflector/joint1_position_controller/command", 1);
  //ros::Publisher pos_cmd_3 = n.advertise<std_msgs::Float64>("/deflector/joint2_position_controller/command", 1); 

  std_msgs::Float64 pos_cmd_float64; //create a variable of type "Float64"
  int num_loops = 300; // Will publish a setpoint for num_loops/loopRate
  int switch_idx = 1;
  double j1_cmd = 0.0;

  // joint displacement limits
  const double j1_lo_limit = 0.0; // prismatic joint 1 lower limit
  const double j1_up_limit = 2.8; // lenght_l1 - x_width_l2

  ROS_INFO("Started deflector_commander node...");

  ros::Rate loopRate(50); // in Hz, in this example loop every 6 seconds
  // run node continuously (which is usefull for this example)
  while (ros::ok()) {
    switch (switch_idx) {
    case 1:
      j1_cmd = remap(0.25, 0.0, 1.0, j1_lo_limit, j1_up_limit);
      switch_idx = 2;
      break;
    case 2:
      j1_cmd = remap(0.5, 0.0, 1.0, j1_lo_limit, j1_up_limit);
      switch_idx = 3;
      break;
    case 3:
      j1_cmd = remap(0.75, 0.0, 1.0, j1_lo_limit, j1_up_limit);
      switch_idx = 1;
      break;
    default:
      printf("case 'switch_idx' not known\n");
      break;
    }
    ROS_INFO("Sending target position to joint 1: %.2f", j1_cmd);
    pos_cmd_float64.data = j1_cmd;

    for (int i = 0; i < num_loops; i++) // Publish data for num_loops/loopRate seconds
    {
      pos_cmd_joint1.publish(pos_cmd_float64); // publish the value--of type Float64-- 
      loopRate.sleep();
    }
  } // end of while loop

}
