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
TalonSRX *dof1MotorPtr;
TalonSRX *dof2MotorPtr;
TalonSRX *dof3MotorPtr;
TalonSRX *dof4MotorPtr;

/* encoders */
CANCoder *dof1SensorPtr;
CANCoder *dof2SensorPtr;
CANCoder *dof3SensorPtr;

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
  	dof1MotorPtr->ConfigClosedloopRamp(0.75);
  	dof1MotorPtr->ConfigAllowableClosedloopError(0, 7);
  	dof1MotorPtr->Config_kP(0, 1);
    dof1MotorPtr->Config_kD(0, 5);
  	double dof1_kI = 0.002;
  	dof1MotorPtr->Config_kI(0, dof1_kI);
  	dof1MotorPtr->Config_IntegralZone(0, 200.0);
  	dof1MotorPtr->ConfigMaxIntegralAccumulator(0, 1023.0 / (10.0 * dof1_kI)); // max 10% driven by I term

  	
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
  	dof2MotorPtr->ConfigClosedloopRamp(1.2);
  	dof2MotorPtr->ConfigAllowableClosedloopError(0, 10);
    dof2MotorPtr->Config_kP(0, 1.0);
    dof2MotorPtr->Config_kD(0, 20.0);
 	dof2MotorPtr->Config_kI(0, 0.006);
 	dof2MotorPtr->Config_IntegralZone(0, 250.0);
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
    dof1SensorPtr->ConfigMagnetOffset(152.8418);
    dof1SensorPtr->ConfigSensorDirection(false);
    dof1SensorPtr->SetPositionToAbsolute();
  	
    dof2SensorPtr->ConfigFactoryDefault();
    dof2SensorPtr->ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
	dof2SensorPtr->ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
	dof2SensorPtr->ConfigMagnetOffset(-54.32);
	dof2SensorPtr->ConfigSensorDirection(true);
	dof2SensorPtr->SetPositionToAbsolute();

	dof3SensorPtr->ConfigFactoryDefault();
	dof3SensorPtr->ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
	dof3SensorPtr->ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
	dof3SensorPtr->ConfigMagnetOffset(92.256);
	dof3SensorPtr->ConfigSensorDirection(false);
    dof3SensorPtr->SetPositionToAbsolute();
}

int main() {
	ctre::phoenix::platform::can::SetCANInterface(INTERFACE.c_str());
	std::cout.precision(2);
	// Comment out the call if you would rather use the automatically running diag-server, note this requires uninstalling diagnostics from Tuner. 
	// c_SetPhoenixDiagnosticsStartTime(-1); // disable diag server, instead we will use the diag server stand alone application that Tuner installs

	/* init */
	init();
	
	kG2 = 0.2;
	kG3 = 0.05;
	printCurrent = false;
	kS2 = 0.01;
	kS3 = 0.04;
	
	std::thread([&] {
	    while(true) {
	        ctre::phoenix::unmanaged::FeedEnable(1000);
	        motorMtx.lock();
	        // add dof 3 FF
	        if (dof3MotorPtr->GetControlMode() == ControlMode::Position) {
	            double tgt = dof3MotorPtr->GetClosedLoopTarget();
	            double ff = kG3 * sin(d2r(dof3SensorPtr->GetPosition()));
	            // cout << "pos tgt: " << fixed << tgt << " arb FF: " << fixed << ff << endl;
	            dof3MotorPtr->Set(ControlMode::Position, tgt, DemandType::DemandType_ArbitraryFeedForward, ff + copysign(kS3, dof3MotorPtr->GetClosedLoopError()));
	        }
	        // add dof 2 FF
	        if (dof2MotorPtr->GetControlMode() == ControlMode::Position) {
	            double tgt = dof2MotorPtr->GetClosedLoopTarget();
	            double q1 = 90 + dof2SensorPtr->GetPosition();
	            double q2 = 180 - dof3SensorPtr->GetPosition();
	            double mass_x = 12 * cos (d2r(q1 + q2)) + 19 * cos (d2r(q1));
	            double mass_y = 12 * sin (d2r(q1 + q2)) + 19 * sin (d2r(q1));
	            double mass_angle = atan2(abs(mass_y), mass_x);
	            double mass_dist = sqrt(mass_x * mass_x + mass_y * mass_y);
	            double ff = kG2 * (mass_dist/31.0) * cos(mass_angle) + /* point mass ratio 1:2 */ 0.1 * kG2 * cos(d2r(q1));
	            // cout << "pos tgt: " << fixed << tgt << " arb FF: " << fixed << ff << endl;
	            dof2MotorPtr->Set(ControlMode::Position, tgt, DemandType::DemandType_ArbitraryFeedForward, ff + copysign(kS2, dof2MotorPtr->GetClosedLoopError()));
	        }
	        if (printCurrent) {
	            cout << "dof3 current is " << fixed << dof3MotorPtr->GetStatorCurrent() * 1000.0 << " mA" << endl;
	        }
	        motorMtx.unlock();
	        std::this_thread::sleep_for(std::chrono::milliseconds(50));
	    }
	}).detach();
	
	TalonSRX* motor = dof2MotorPtr;
    cout << "selecting dof 2" << endl;
	while(true) {
        std::string line;
        std::getline(std::cin, line);
        std::string cmd;
        double param;
        stringstream(line) >> cmd >> param;
        motorMtx.lock();
        cout.precision(2);
        if (cmd == "dof") {
            int dof = (int) param;
            cout << "selecting dof " << dof << endl;
            switch (dof) {
            case 1:
                motor = dof1MotorPtr;
                break;
            case 2:
                motor = dof2MotorPtr;
                break;
            case 3:
                motor = dof3MotorPtr;
                break;
            case 4:
                motor = dof4MotorPtr;
                break;
            default:
                break;
            }
        }
        else if (cmd == "coast") {
	        motor->SetNeutralMode(motorcontrol::NeutralMode::Coast);
        }
        else if (cmd == "brake") {
	        motor->SetNeutralMode(motorcontrol::NeutralMode::Brake);	
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
            cout.precision(5);
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
        else if (cmd == "getpos") {
            // get pos
            cout << "position is " << std::fixed << 360.0/4096.0 * motor->GetSelectedSensorPosition() << " degrees" << endl;
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
