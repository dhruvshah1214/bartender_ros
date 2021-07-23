#ifndef BARTENDER_CONTROL__HW_INTERFACE_H
#define BARTENDER_CONTROL__HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>

namespace bartender_control {
  class BartenderHWInterface : public ros_control_boilerplate::GenericHWInterface {
  public:
    BartenderHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);
    
    virtual void read(ros::Duration& elapsed_time);
    
    virtual void write(ros::Duration& elapsed_time);
    
    virtual void enforceLimits(ros::Duration& period);

  private:
    
  };
}

#endif
