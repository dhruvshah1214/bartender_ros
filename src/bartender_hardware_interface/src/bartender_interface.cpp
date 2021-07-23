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

/* actuators */
TalonSRX dof1Motor(11);
TalonSRX dof2Motor(12);
TalonSRX dof3Motor(13);
TalonSRX dof4Motor(14);

/* encoders */
CANCoder dof1Sensor(21);
CANCoder dof2Sensor(22);
CANCoder dof3Sensor(23);


void init_ctre()
{
	// talons

	dof1Motor.ConfigFactoryDefault();
	dof1Motor.SetNeutralMode(motorcontrol::NeutralMode::Brake);	
	dof1Motor.SetInverted(true);
	dof1Motor.ConfigRemoteFeedbackFilter(dof1Sensor, 0);
	dof1Motor.ConfigSensorTerm(SensorTerm::SensorTerm_Diff0, FeedbackDevice::None);
	dof1Motor.ConfigSensorTerm(SensorTerm::SensorTerm_Diff1, FeedbackDevice::RemoteSensor0);
	dof1Motor.ConfigSelectedFeedbackSensor(FeedbackDevice::SensorDifference, 0);
	dof1Motor.ConfigPeakOutputForward(0.4);
	dof1Motor.ConfigPeakOutputReverse(-0.4);
	dof1Motor.ConfigForwardSoftLimitEnable(true);
	dof1Motor.ConfigReverseSoftLimitEnable(true);
	dof1Motor.ConfigForwardSoftLimitThreshold(120.0 * (4096.0/360.0));
	dof1Motor.ConfigReverseSoftLimitThreshold(-120.0 * (4096.0/360.0));
  dof1Motor.ConfigClosedLoopPeakOutput(0, 0.3);
	// dof1Motor.ConfigMotionCruiseVelocity(75.0);
	// dof1Motor.ConfigMotionAcceleration(225.0);
	// dof1Motor.ConfigMotionSCurveStrength(4);
	// dof1Motor.Config_kF(0, 5.115);
	dof1Motor.Config_kP(0, 2.0);

	
	
	// sensors
	
	dof1Sensor.ConfigFactoryDefault();
	dof1Sensor.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
	dof1Sensor.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
	dof1Sensor.ConfigMagnetOffset(0);
	dof1Sensor.ConfigSensorDirection(false);
	dof1Sensor.SetPositionToAbsolute();
}

namespace bartender_control {
  BartenderHWInterface::BartenderHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model) : ros_control_boilerplate::GenericHWInterface(nh, urdf_model) {
    ROS_INFO_NAMED("bartender_interface", "hw interface ready");
    ctre::phoenix::platform::can::SetCANInterface(INTERFACE.c_str());
    init_ctre();
  }
  
  void BartenderHWInterface::read(ros::Duration& elapsed_time) {
    joint_position_[0] = dof1Sensor.GetAbsolutePosition();
    joint_velocity_[0] = dof1Sensor.GetVelocity();
  }
  
  void BartenderHWInterface::write(ros::Duration& elapsed_time) {
    dof1Motor.Set(TalonSRXControlMode::Position, joint_position_command_[0] * (4096.0/360.0));
  }
  
  void BartenderHWInterface::enforceLimits(ros::Duration& period) {
  }
  
}
