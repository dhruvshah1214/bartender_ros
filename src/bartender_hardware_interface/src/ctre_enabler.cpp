#include "ros/ros.h"

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include <string>
#include <iostream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "ctre_enabler");
  
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  
  while(ros::ok()) {
    ctre::phoenix::unmanaged::FeedEnable(1000);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
