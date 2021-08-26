#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include <string>
#include <iostream>

#include <bartender_hardware_interface/bartender_interface.h>

#include "ros/ros.h"
#include <std_srvs/Empty.h>

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

// control constants

double kS2 = 0.0;
double kS3 = 0.0;
double kG2 = 0.0;
double kG3 = 0.0;

// util

#define d2r(degrees) degrees * M_PI / 180.0
#define r2d(rad) rad * 180.0 / M_PI

// enable

bool enabled = true;

void init_ctre() {

  /* actuators */
    
  // talons
  dof1MotorPtr = new TalonSRX(11);
  dof2MotorPtr = new TalonSRX(12);
  dof3MotorPtr = new TalonSRX(13);
  
    /* encoders */
    dof1SensorPtr = new CANCoder(21);
    dof2SensorPtr = new CANCoder(22);
    dof3SensorPtr = new CANCoder(23);
  	
  
  	dof1MotorPtr->ConfigFactoryDefault();
  	dof1MotorPtr->SetNeutralMode(motorcontrol::NeutralMode::Coast);	
  	dof1MotorPtr->SetInverted(true);
  	dof1MotorPtr->ConfigRemoteFeedbackFilter(*dof1SensorPtr, 0);
  	dof1MotorPtr->ConfigSensorTerm(SensorTerm::SensorTerm_Diff0, FeedbackDevice::None);
  	dof1MotorPtr->ConfigSensorTerm(SensorTerm::SensorTerm_Diff1, FeedbackDevice::RemoteSensor0);
  	dof1MotorPtr->ConfigSelectedFeedbackSensor(FeedbackDevice::SensorDifference, 0);
  	dof1MotorPtr->ConfigNeutralDeadband(0.01);
  	dof1MotorPtr->ConfigPeakOutputForward(0.4);
  	dof1MotorPtr->ConfigPeakOutputReverse(-0.4);
  	dof1MotorPtr->ConfigForwardSoftLimitEnable(true);
  	dof1MotorPtr->ConfigReverseSoftLimitEnable(true);
  	dof1MotorPtr->ConfigForwardSoftLimitThreshold(120.0 * (4096.0/360.0));
  	dof1MotorPtr->ConfigReverseSoftLimitThreshold(-120.0 * (4096.0/360.0));
  	dof1MotorPtr->ConfigClosedloopRamp(1.2);
  	dof1MotorPtr->ConfigAllowableClosedloopError(0, 7);
  	dof1MotorPtr->Config_kP(0, 0.4);
  	dof1MotorPtr->Config_kD(0, 1.2);
  	dof1MotorPtr->Config_kI(0, 0.0052);
  	dof1MotorPtr->Config_IntegralZone(0, 200.0);
  	dof1MotorPtr->ConfigMaxIntegralAccumulator(0, 5000.0);

  	
  dof2MotorPtr->ConfigFactoryDefault();
  dof2MotorPtr->SetNeutralMode(motorcontrol::NeutralMode::Brake);	
  dof2MotorPtr->SetInverted(false);
  	dof2MotorPtr->ConfigRemoteFeedbackFilter(*dof2SensorPtr, 0);
  dof2MotorPtr->ConfigSensorTerm(SensorTerm::SensorTerm_Sum0, FeedbackDevice::RemoteSensor0);
  dof2MotorPtr->ConfigSensorTerm(SensorTerm::SensorTerm_Sum1, FeedbackDevice::None);
  	dof2MotorPtr->ConfigSelectedFeedbackSensor(FeedbackDevice::SensorSum, 0);
  dof2MotorPtr->ConfigNeutralDeadband(0.01);
  dof2MotorPtr->ConfigPeakOutputForward(0.7);
  dof2MotorPtr->ConfigPeakOutputReverse(-0.7);
  dof2MotorPtr->ConfigForwardSoftLimitEnable(true);
  	dof2MotorPtr->ConfigReverseSoftLimitEnable(true);
  	dof2MotorPtr->ConfigForwardSoftLimitThreshold(45.0 * (4096.0/360.0));
  	dof2MotorPtr->ConfigReverseSoftLimitThreshold(-45.0 * (4096.0/360.0));
  	dof2MotorPtr->ConfigClosedloopRamp(0.8);
  	dof2MotorPtr->ConfigAllowableClosedloopError(0, 10);
  dof2MotorPtr->Config_kP(0, 1.5);
  dof2MotorPtr->Config_kD(0, 5.0);
 	dof2MotorPtr->Config_kI(0, 0.008);
 	dof2MotorPtr->Config_IntegralZone(0, 100.0);
 	dof2MotorPtr->ConfigMaxIntegralAccumulator(0, 20000.0);

    dof3MotorPtr->ConfigFactoryDefault();
  	dof3MotorPtr->SetNeutralMode(motorcontrol::NeutralMode::Brake);	
 	dof3MotorPtr->SetInverted(true);
 	dof3MotorPtr->ConfigRemoteFeedbackFilter(*dof3SensorPtr, 0);
 	dof3MotorPtr->ConfigSensorTerm(SensorTerm::SensorTerm_Diff0, FeedbackDevice::None);
 	dof3MotorPtr->ConfigSensorTerm(SensorTerm::SensorTerm_Diff1, FeedbackDevice::RemoteSensor0);
 	dof3MotorPtr->ConfigSelectedFeedbackSensor(FeedbackDevice::SensorDifference, 0);
  	dof3MotorPtr->ConfigNeutralDeadband(0.01);
  	dof3MotorPtr->ConfigPeakOutputForward(0.6);
  	dof3MotorPtr->ConfigPeakOutputReverse(-0.6);
  	dof3MotorPtr->ConfigForwardSoftLimitEnable(true);
  	dof3MotorPtr->ConfigReverseSoftLimitEnable(true);
  	dof3MotorPtr->ConfigForwardSoftLimitThreshold(150.0 * (4096.0/360.0));
  	dof3MotorPtr->ConfigReverseSoftLimitThreshold(60.0 * (4096.0/360.0));
  	dof3MotorPtr->ConfigClosedloopRamp(0.35);
  	dof3MotorPtr->ConfigAllowableClosedloopError(0, 10.0);
  	dof3MotorPtr->Config_kP(0, 1.5);
  	dof3MotorPtr->Config_kD(0, 4.0);
  	dof3MotorPtr->Config_kI(0, 0.01);
  	dof3MotorPtr->Config_IntegralZone(0, 250.0);
  	dof3MotorPtr->ConfigMaxIntegralAccumulator(0, 20000.0);
	
  	// sensors
  	
  dof1SensorPtr->ConfigFactoryDefault();
  dof1SensorPtr->ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
  dof1SensorPtr->ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  dof1SensorPtr->ConfigMagnetOffset(-57.216);
  dof1SensorPtr->ConfigSensorDirection(false);
  dof1SensorPtr->SetPositionToAbsolute();
  	
  dof2SensorPtr->ConfigFactoryDefault();
  dof2SensorPtr->ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
	dof2SensorPtr->ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
	dof2SensorPtr->ConfigMagnetOffset(184.482);
	dof2SensorPtr->ConfigSensorDirection(true);
	dof2SensorPtr->SetPositionToAbsolute();

	dof3SensorPtr->ConfigFactoryDefault();
	dof3SensorPtr->ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
	dof3SensorPtr->ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
	dof3SensorPtr->ConfigMagnetOffset(-25.3);
	dof3SensorPtr->ConfigSensorDirection(false);
  dof3SensorPtr->SetPositionToAbsolute();
	
}

namespace bartender_control {
  BartenderHWInterface::BartenderHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model) : ros_control_boilerplate::GenericHWInterface(nh, urdf_model) {
    serviceNeutral = nh.advertiseService("neutral", &bartender_control::BartenderHWInterface::neutral, this);
    serviceEnable = nh.advertiseService("enable", &bartender_control::BartenderHWInterface::enable, this);
    enabled = false;
    
    ctre::phoenix::platform::can::SetCANInterface(INTERFACE.c_str());
    init_ctre();
    
    kS3 = 0.04;
    kG3 = 0.05;
    kS2 = 0.01;
    kG2 = 0.20;
  }
  
  void BartenderHWInterface::init() {
    ros_control_boilerplate::GenericHWInterface::init();
    ROS_INFO_NAMED("bartender_interface", "hw interface ready");
    
    joint_position_command_[0] = 0.0;
    joint_position_command_[1] = 0.0;
    joint_position_command_[2] = 60.0;
    
    enabled = false;
  }
  
  void BartenderHWInterface::read(ros::Duration& elapsed_time) {
    // ROS_INFO_NAMED("bartender_interface", "reading position:");
    joint_position_[0] = dof1SensorPtr->GetPosition();
    //ROS_INFO_NAMED("bartender_interface", std::to_string(joint_position_[0]).c_str());
    joint_velocity_[0] = dof1SensorPtr->GetVelocity();
    
    joint_position_[1] = dof2SensorPtr->GetPosition();
    joint_velocity_[1] = dof2SensorPtr->GetVelocity();
    
    joint_position_[2] = dof3SensorPtr->GetPosition();
    joint_velocity_[2] = dof3SensorPtr->GetVelocity();
  }
  
  void BartenderHWInterface::write(ros::Duration& elapsed_time) {
    // ROS_INFO_NAMED("bartender_interface", "writing position:");
    // ROS_INFO_NAMED("bartender_interface", std::to_string(joint_position_command_[0]).c_str());
    if (enabled) {
      ctre::phoenix::unmanaged::FeedEnable(1000);
      dof1MotorPtr->Set(ControlMode::Position, joint_position_command_[0] * (4096.0/360.0));
	    double mass_x = 12 * cos (d2r(270 + joint_position_[1] - joint_position_[2])) + 19 * cos (d2r(90 + joint_position_[1]));
	    double mass_y = 12 * sin (d2r(270 + joint_position_[1] - joint_position_[2])) + 19 * sin (d2r(90 + joint_position_[1]));
	    double mass_angle = atan2(abs(mass_y), mass_x);
	    double ff2 = kG2 * cos(mass_angle);
      dof2MotorPtr->Set(ControlMode::Position, joint_position_command_[1] * (4096.0/360.0), DemandType::DemandType_ArbitraryFeedForward, ff2 + copysign(kS2, dof2MotorPtr->GetClosedLoopError()));
      double ff3 = kG3 * sin(d2r(joint_position_[2]));
      dof3MotorPtr->Set(ControlMode::Position, joint_position_command_[2] * (4096.0/360.0), DemandType::DemandType_ArbitraryFeedForward, ff3 + copysign(kS3, dof3MotorPtr->GetClosedLoopError()));
    }
    
  }
  
  void BartenderHWInterface::enforceLimits(ros::Duration& period) {
  }
  
  bool BartenderHWInterface::neutral(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res) {
  dof1MotorPtr->NeutralOutput();
  dof2MotorPtr->NeutralOutput();  
  dof3MotorPtr->NeutralOutput();

  enabled = false;
  return true;
}

  bool BartenderHWInterface::enable(std_srvs::Empty::Request &req,
                                    std_srvs::Empty::Response &res) {
    enabled = true;
    return true;
  }
  
}
