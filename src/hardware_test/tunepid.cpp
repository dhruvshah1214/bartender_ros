#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

#include <string>
#include <string.h>
#include <cmath>

#include <iostream>

#include <chrono>
#include <thread>
#include <mutex>



using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::sensors;


using namespace std;

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

// motor thread lock for FF
std::mutex motorMtx;


// state

double kG3 = 0.0;
double kG2 = 0.0;
bool printCurrent = false;
double kS3 = 0.0;
double kS2 = 0.0;

// util

double d2r(double degrees) {
    return degrees * M_PI / 180.0;
}

double r2d(double rad) {
    return rad * 180.0 / M_PI;
}


// code

void init()
{
	// talons

	
	dof1Motor.ConfigFactoryDefault();
	dof1Motor.SetNeutralMode(motorcontrol::NeutralMode::Coast);	
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
	dof2Motor.ConfigPeakOutputForward(0.7);
	dof2Motor.ConfigPeakOutputReverse(-0.7);
	dof2Motor.ConfigForwardSoftLimitEnable(true);
	dof2Motor.ConfigReverseSoftLimitEnable(true);
	dof2Motor.ConfigForwardSoftLimitThreshold(45.0 * (4096.0/360.0));
	dof2Motor.ConfigReverseSoftLimitThreshold(-45.0 * (4096.0/360.0));
	dof2Motor.ConfigMotionCruiseVelocity(75.0);
	dof2Motor.ConfigMotionAcceleration(225.0);
	dof2Motor.ConfigMotionSCurveStrength(4);
	// dof2Motor.Config_kP(0, 0.5);

	dof3Motor.ConfigFactoryDefault();
	dof3Motor.SetNeutralMode(motorcontrol::NeutralMode::Brake);	
	dof3Motor.SetInverted(true);
	dof3Motor.ConfigRemoteFeedbackFilter(dof3Sensor, 0);
	dof3Motor.ConfigSensorTerm(SensorTerm::SensorTerm_Diff0, FeedbackDevice::None);
	dof3Motor.ConfigSensorTerm(SensorTerm::SensorTerm_Diff1, FeedbackDevice::RemoteSensor0);
	dof3Motor.ConfigSelectedFeedbackSensor(FeedbackDevice::SensorDifference, 0);
	dof3Motor.ConfigPeakOutputForward(0.6);
	dof3Motor.ConfigPeakOutputReverse(-0.6);
	dof3Motor.ConfigForwardSoftLimitEnable(true);
	dof3Motor.ConfigReverseSoftLimitEnable(true);
	dof3Motor.ConfigForwardSoftLimitThreshold(150.0 * (4096.0/360.0));
	dof3Motor.ConfigReverseSoftLimitThreshold(60.0 * (4096.0/360.0));
	dof3Motor.ConfigMotionCruiseVelocity(75.0);
	dof3Motor.ConfigMotionAcceleration(225.0);
	dof3Motor.ConfigMotionSCurveStrength(4);
	
	// sensors
	
	dof1Sensor.ConfigFactoryDefault();
	dof1Sensor.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
	dof1Sensor.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
	dof1Sensor.ConfigMagnetOffset(-57);
	dof1Sensor.ConfigSensorDirection(false);
	dof1Sensor.SetPositionToAbsolute();

	dof2Sensor.ConfigFactoryDefault();
	dof2Sensor.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
	dof2Sensor.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
	dof2Sensor.ConfigMagnetOffset(184.482);
	dof2Sensor.ConfigSensorDirection(true);
	dof2Sensor.SetPositionToAbsolute();

	dof3Sensor.ConfigFactoryDefault();
	dof3Sensor.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
	dof3Sensor.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
	dof3Sensor.ConfigMagnetOffset(-25.3);
	dof3Sensor.ConfigSensorDirection(false);
	dof3Sensor.SetPositionToAbsolute();
}

int main() {
	ctre::phoenix::platform::can::SetCANInterface(INTERFACE.c_str());
	std::cout.precision(2);
	// Comment out the call if you would rather use the automatically running diag-server, note this requires uninstalling diagnostics from Tuner. 
	// c_SetPhoenixDiagnosticsStartTime(-1); // disable diag server, instead we will use the diag server stand alone application that Tuner installs

	/* init */
	init();
	
	kG3 = 0.0;
	kG2 = 0.0;
	printCurrent = false;
	kS2 = 0.0;
	kS3 = 0.0;
	
	std::thread([&] {
	    while(true) {
	        ctre::phoenix::unmanaged::FeedEnable(1000);
	        motorMtx.lock();
	        // add dof 3 FF
	        if (dof3Motor.GetControlMode() == ControlMode::Position) {
	            double tgt = dof3Motor.GetClosedLoopTarget();
	            double ff = kG3 * sin(d2r(dof3Sensor.GetPosition()));
	            // cout << "pos tgt: " << fixed << tgt << " arb FF: " << fixed << ff << endl;
	            dof3Motor.Set(ControlMode::Position, tgt, DemandType::DemandType_ArbitraryFeedForward, ff + copysign(kS3, dof3Motor.GetClosedLoopError()));
	        }
	        // add dof 2 FF
	        if (dof2Motor.GetControlMode() == ControlMode::Position) {
	            double tgt = dof2Motor.GetClosedLoopTarget();
	            double q1 = 90 + dof2Sensor.GetPosition();
	            double q2 = 180 - dof3Sensor.GetPosition();
	            double mass_x = 12 * cos (d2r(q1 + q2)) + 19 * cos (d2r(q1));
	            double mass_y = 12 * sin (d2r(q1 + q2)) + 19 * sin (d2r(q1));
	            double mass_angle = atan2(abs(mass_y), mass_x);
	            double ff = kG2 * cos(mass_angle);
	            // cout << "pos tgt: " << fixed << tgt << " arb FF: " << fixed << ff << endl;
	            dof2Motor.Set(ControlMode::Position, tgt, DemandType::DemandType_ArbitraryFeedForward, ff + copysign(kS2, dof2Motor.GetClosedLoopError()));
	        }
	        if (printCurrent) {
	            cout << "dof3 current is " << fixed << dof3Motor.GetStatorCurrent() * 1000.0 << " mA" << endl;
	        }
	        motorMtx.unlock();
	        std::this_thread::sleep_for(std::chrono::milliseconds(50));
	    }
	}).detach();
	
	TalonSRX* motor = &dof2Motor;
    cout << "selecting dof 2" << endl;
	while(true) {
        std::string line;
        std::getline(std::cin, line);
        std::string cmd;
        double param;
        stringstream(line) >> cmd >> param;
        motorMtx.lock();
        if (cmd == "dof") {
            int dof = (int) param;
            cout << "selecting dof " << dof << endl;
            switch (dof) {
            case 1:
                motor = &dof1Motor;
                break;
            case 2:
                motor = &dof2Motor;
                break;
            case 3:
                motor = &dof3Motor;
                break;
            case 4:
                motor = &dof4Motor;
                break;
            default:
                break;
            }
        }
        else if (cmd == "coast") {
	        dof1Motor.SetNeutralMode(motorcontrol::NeutralMode::Coast);
	        dof2Motor.SetNeutralMode(motorcontrol::NeutralMode::Coast);	
	        dof3Motor.SetNeutralMode(motorcontrol::NeutralMode::Coast);	
            dof4Motor.SetNeutralMode(motorcontrol::NeutralMode::Coast);	
        }
        else if (cmd == "brake") {
	        dof1Motor.SetNeutralMode(motorcontrol::NeutralMode::Brake);
	        dof2Motor.SetNeutralMode(motorcontrol::NeutralMode::Brake);	
	        dof3Motor.SetNeutralMode(motorcontrol::NeutralMode::Brake);	
            dof4Motor.SetNeutralMode(motorcontrol::NeutralMode::Brake);	
        }
        else if(cmd == "getp") {
            ctre::phoenix::motorcontrol::can::SlotConfiguration c;
            motor->GetSlotConfigs(c);
            cout << "p is " << c.kP << endl;
        }
        else if (cmd == "setp") {
            // set p
            cout << "setting p to " << param << endl;
            motor->Config_kP(0, param);
        }
        else if (cmd == "geti") {
            ctre::phoenix::motorcontrol::can::SlotConfiguration c;
            motor->GetSlotConfigs(c);
            cout << "i is " << c.kI << endl;
        }
        else if (cmd == "seti") {
            // set i
            cout << "setting i to " << param << endl;
            motor->Config_kI(0, param);
        }
        else if (cmd == "getd") {
            ctre::phoenix::motorcontrol::can::SlotConfiguration c;
            motor->GetSlotConfigs(c);
            cout << "d is " << c.kD << endl;
        }
        else if (cmd == "setd") {
            // set d
            cout << "setting d to " << param << endl;
            motor->Config_kD(0, param);
        }
        else if(cmd == "getf") {
            ctre::phoenix::motorcontrol::can::SlotConfiguration c;
            motor->GetSlotConfigs(c);
            cout << "ff is " << c.kF << endl;
        }
        else if (cmd == "setf") {
            // set kf
            cout << "setting kF to " << param << endl;
            motor->Config_kF(0, param);
        }
        else if (cmd == "gets3") {
            cout << "kS3 is " << kS3 << endl;
        }
        else if (cmd == "sets3") {
            // set kS3
            cout << "setting kS3 to " << param << endl;
            kS3 = param;
        }
        else if (cmd == "gets2") {
            cout << "kS2 is " << kS2 << endl;
        }
        else if (cmd == "sets2") {
            // set kS2
            cout << "setting kS2 to " << param << endl;
            kS2 = param;
        }
        else if (cmd == "getg3") {
            cout << "kG3 is " << kG3 << endl;
        }
        else if (cmd == "setg3") {
            // set kG3
            cout << "setting kG3 to " << param << endl;
            kG3 = param;
        }
        else if (cmd == "getg2") {
            cout << "kG2 is " << kG2 << endl;
        }
        else if (cmd == "setg2") {
            // set kG2
            cout << "setting kG2 to " << param << endl;
            kG2 = param;
        }
        else if (cmd == "err") {
            // get closed loop err
            double erru = motor->GetClosedLoopError();
            cout << "closed loop error is " << fixed  << erru * 360.0/4096.0 << "degrees, and " << erru << " units" << endl;
        }
        else if (cmd == "iacc") {
            // get iacc
            double erru = motor->GetIntegralAccumulator();
            cout << "iacc is " << fixed  << erru << endl;
        }
        else if (cmd == "printcur") {
            // get current
            printCurrent = ((int) param) == 1;
        }
        else if (cmd == "getbusv") {
            cout << "bus voltage " << fixed  << motor->GetBusVoltage() << endl;
        }
        else if (cmd == "pow") {
            // get power
            cout << "motor power is " << fixed  << 100.0 * motor->GetMotorOutputPercent() << "%" << endl;
        }
        else if (cmd == "pos") {
            // cmd pos
            double runit = ((double) param) * 4096.0/360.0;
            cout << "commanding position of " << param << " degrees, raw units " << std::fixed << runit << endl;
            motor->Set(ControlMode::Position, runit);
        }
        else if (cmd == "cur") {
            // cmd pos
            double amp = ((double) param) / 1000.0;
            cout << "commanding " << amp << " A" << endl;
            motor->Set(ControlMode::Current, amp);
        }
        else {
            if(std::isdigit(cmd[0]) || (cmd[0] == '-' && std::isdigit(cmd[1]))) {
                stringstream(cmd) >> param;
                // set % output to param
                double pct = ((double) param) / 100.0;
                cout << "setting % output to " << std::fixed << pct << endl;
                motor->Set(ControlMode::PercentOutput, pct);
            }
            else {
                // set neutral output
                cout << "disabling!" << endl;
                motor->NeutralOutput();
            }
        }
        motorMtx.unlock();
    }
	return 0;
}
