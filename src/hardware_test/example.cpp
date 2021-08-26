#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

#include "socketcan_cpp/socketcan_cpp.h"

#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <string.h>

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::sensors;

using namespace scpp;

typedef uint8_t byte;


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

void init()
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
	dof1Motor.ConfigMotionCruiseVelocity(75.0);
	dof1Motor.ConfigMotionAcceleration(225.0);
	dof1Motor.ConfigMotionSCurveStrength(4);
	dof1Motor.Config_kP(0, 0.5);

	dof2Motor.ConfigFactoryDefault();
	dof2Motor.SetNeutralMode(motorcontrol::NeutralMode::Brake);	
	dof2Motor.SetInverted(false);
	dof2Motor.ConfigRemoteFeedbackFilter(dof2Sensor, 0);
	dof2Motor.ConfigSensorTerm(SensorTerm::SensorTerm_Sum0, FeedbackDevice::RemoteSensor0);
	dof2Motor.ConfigSensorTerm(SensorTerm::SensorTerm_Sum1, FeedbackDevice::None);
	dof2Motor.ConfigSelectedFeedbackSensor(FeedbackDevice::SensorSum, 0);
	dof2Motor.ConfigPeakOutputForward(0.5);
	dof2Motor.ConfigPeakOutputReverse(-0.5);
	dof2Motor.ConfigForwardSoftLimitEnable(true);
	dof2Motor.ConfigReverseSoftLimitEnable(true);
	dof2Motor.ConfigForwardSoftLimitThreshold(45.0 * (4096.0/360.0));
	dof2Motor.ConfigReverseSoftLimitThreshold(-45.0 * (4096.0/360.0));
	dof2Motor.ConfigMotionCruiseVelocity(75.0);
	dof2Motor.ConfigMotionAcceleration(225.0);
	dof2Motor.ConfigMotionSCurveStrength(4);
	// dof2Motor.Config_kP(0, 0.5);

	
	
	// sensors
	
	dof1Sensor.ConfigFactoryDefault();
	dof1Sensor.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
	dof1Sensor.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
	dof1Sensor.ConfigMagnetOffset(-144.316);
	dof1Sensor.ConfigSensorDirection(false);
	dof1Sensor.SetPositionToAbsolute();

	dof2Sensor.ConfigFactoryDefault();
	dof2Sensor.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
	dof2Sensor.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
	dof2Sensor.ConfigMagnetOffset(3.867);
	dof2Sensor.ConfigSensorDirection(true);
	dof2Sensor.SetPositionToAbsolute();
}

int main() {
	ctre::phoenix::platform::can::SetCANInterface(INTERFACE.c_str());
	
	
	SocketCan socketCan;
	if (socketCan.open("can0") == scpp::STATUS_OK) {
        printf("SocketCAN opened successfully!\n");    
	}
	else {
	    printf("SocketCAN failed");
	}
	
	// Comment out the call if you would rather use the automatically running diag-server, note this requires uninstalling diagnostics from Tuner. 
	// c_SetPhoenixDiagnosticsStartTime(-1); // disable diag server, instead we will use the diag server stand alone application that Tuner installs

	/* init */
	init();

/*	
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	
	float pwr = 1.0;

	scpp::CanFrame cf;
    cf.id = 0x1F6;
    cf.len = 4;
    memcpy(cf.data, &pwr, sizeof(pwr));
    auto write_sc_status = socketCan.write(cf);
    if (write_sc_status != scpp::STATUS_OK)
        printf("something went wrong on socket write, error code : %d\n", int32_t(write_sc_status));
    else
        printf("Message was written to the socket!\n");

*/
	//dof4Motor.Set(ControlMode::PercentOutput, 0.5);
	//ctre::phoenix::unmanaged::FeedEnable(10000);
	while(true){
	}

	return 0;
}
