#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include <string>
#include <iostream>

#include <bartender_hardware_interface/bartender_interface.h>

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::sensors;

// const
const std::string INTERFACE = "can0";

TalonSRX *dof1MotorPtr;
TalonSRX *dof2MotorPtr;
TalonSRX *dof3MotorPtr;
TalonSRX *dof4MotorPtr;

CANCoder *dof1SensorPtr;
CANCoder *dof2SensorPtr;
CANCoder *dof3SensorPtr;

  void init_ctre() {

    /* actuators */
    
    // talons
    dof1MotorPtr = new TalonSRX(11);
    dof2MotorPtr = new TalonSRX(12);
    //TalonSRX dof3Motor(13);
    //TalonSRX dof4Motor(14);
  
    /* encoders */
    dof1SensorPtr = new CANCoder(21);
    //CANCoder dof2Sensor(22);
    //CANCoder dof3Sensor(23);
  	
  
  	dof1MotorPtr->ConfigFactoryDefault();
  	dof1MotorPtr->SetNeutralMode(motorcontrol::NeutralMode::Brake);	
  	dof1MotorPtr->SetInverted(true);
  	dof1MotorPtr->ConfigRemoteFeedbackFilter(*dof1SensorPtr, 0);
  	dof1MotorPtr->ConfigSensorTerm(SensorTerm::SensorTerm_Diff0, FeedbackDevice::None);
  	dof1MotorPtr->ConfigSensorTerm(SensorTerm::SensorTerm_Diff1, FeedbackDevice::RemoteSensor0);
  	dof1MotorPtr->ConfigSelectedFeedbackSensor(FeedbackDevice::SensorDifference, 0);
  	dof1MotorPtr->ConfigPeakOutputForward(0.4);
  	dof1MotorPtr->ConfigPeakOutputReverse(-0.4);
  	dof1MotorPtr->ConfigForwardSoftLimitEnable(true);
  	dof1MotorPtr->ConfigReverseSoftLimitEnable(true);
  	dof1MotorPtr->ConfigForwardSoftLimitThreshold(120.0 * (4096.0/360.0));
  	dof1MotorPtr->ConfigReverseSoftLimitThreshold(-120.0 * (4096.0/360.0));
    dof1MotorPtr->ConfigClosedLoopPeakOutput(0, 0.3);
  	// dof1MotorPtr.ConfigMotionCruiseVelocity(75.0);
  	// dof1MotorPtr.ConfigMotionAcceleration(225.0);
  	// dof1MotorPtr.ConfigMotionSCurveStrength(4);
  	// dof1MotorPtr.Config_kF(0, 5.115);
  	dof1MotorPtr->Config_kP(0, 2.0);

	
	
  	// sensors
  	
  	dof1SensorPtr->ConfigFactoryDefault();
  	dof1SensorPtr->ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
  	dof1SensorPtr->ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  	dof1SensorPtr->ConfigMagnetOffset(-145.723);
  	dof1SensorPtr->ConfigSensorDirection(false);
  	dof1SensorPtr->SetPositionToAbsolute();
	
  }

namespace bartender_control {



  BartenderHWInterface::BartenderHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model) : ros_control_boilerplate::GenericHWInterface(nh, urdf_model) {
    ROS_INFO_NAMED("bartender_interface", "hw interface ready");
    ctre::phoenix::platform::can::SetCANInterface(INTERFACE.c_str());
    init_ctre();
  }
  
  void BartenderHWInterface::read(ros::Duration& elapsed_time) {
    // ROS_INFO_NAMED("bartender_interface", "reading position:");
    joint_position_[0] = dof1SensorPtr->GetAbsolutePosition();
    //ROS_INFO_NAMED("bartender_interface", std::to_string(joint_position_[0]).c_str());
    joint_velocity_[0] = dof1SensorPtr->GetVelocity();
  }
  
  void BartenderHWInterface::write(ros::Duration& elapsed_time) {
    // ROS_INFO_NAMED("bartender_interface", "writing position:");
    // ROS_INFO_NAMED("bartender_interface", std::to_string(joint_position_command_[0]).c_str());
    dof1MotorPtr->Set(ControlMode::Position, joint_position_command_[0] * (4096.0/360.0));
    
    ctre::phoenix::unmanaged::FeedEnable(1000);
  }
  
  void BartenderHWInterface::enforceLimits(ros::Duration& period) {
  }
  
}
