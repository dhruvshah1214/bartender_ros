#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <bartender_hardware_interface/bartender_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bartender_hw_interface");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Create the hardware interface specific to your robot
  boost::shared_ptr<bartender_control::BartenderHWInterface> hw_interface(new bartender_control::BartenderHWInterface(nh));
  hw_interface->init();

  // Start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, hw_interface);
  control_loop.run();  // Blocks until shutdown signal recieved

  return 0;
}
