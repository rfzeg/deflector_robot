/*
Exemplary Command Algorithm for the Deflector Robot
Author: Roberto Zegers
Date: 2019-Apr-26
*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "deflector_commander"); // init ROS and set name of this node
    ros::NodeHandle n; // Create a ROS NodeHandle object
    ros::Publisher pos_cmd_joint2 = n.advertise<std_msgs::Float64>("/deflector/joint2_position_controller/command", 1);    
    //ros::Publisher pos_cmd_3 = n.advertise<std_msgs::Float64>("/deflector/joint2_position_controller/command", 1); 

    //create a variable of type "Float64"
    std_msgs::Float64 pos_cmd_float64;
    int a = 1;
    double x_cmd = 0.0;

    ros::Rate rate(0.2); // in Hz, in this example loop every 5 seconds
    // publish in infinite loop (which is usefull for this example)
    while(ros::ok())
    {
        switch(a) 
            {
	        case 1: 
                   x_cmd = 0.5;
                   a = 2;
                   break;
	        case 2: 
                   x_cmd = 1.0;
                   a = 3;
                   break;
	        case 3:      
                   x_cmd = 1.5;
                   a = 4;
                   break;
	        case 4:      
                   x_cmd = 1.9;
                   a = 5;
                   break;
	        case 5:      
                   x_cmd = 0.0;
                   a = 1;
                   break;
	        default: printf("case 'a' not known\n"); break;
            }

        pos_cmd_float64.data = x_cmd;
        pos_cmd_joint2.publish(pos_cmd_float64); // publish the value--of type Float64-- 
        rate.sleep();
    }

}
