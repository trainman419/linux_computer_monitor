/*
 * computer_monitor
 *
 * A node to read lm_sensors on Linux and publish sensor values and diagnostics
 *
 * Author: Austin Hendrix
 */

#include <ros/ros.h>

int main(int argc, char ** argv) {

   ros::init(argc, argv, "computer_monitor");

   ros::NodeHandle nh;

   ros::Rate loop_rate(1.0); // publish at 1Hz

   while( ros::ok() ) {
      ros::spinOnce();
      loop_rate.sleep();
   }
   return 0;
}
