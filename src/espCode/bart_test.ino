#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <CAN.h>


Servo servo4;
AccelStepper stepper5(AccelStepper::DRIVER, 21, 19);


// ------------- CONSTANTS -----------
// stepper const
#define  STEPPER_MAX_SPEED 6000
#define  Stepper_MIN_SPEED 0.1
// CAN const
#define  CAN_BITRATE 1000000
#define  CAN_ID 1

// ----------- pin declarations ---------------------
// CAN
const int PIN_CAN_TX = 17;
const int PIN_CAN_RX = 16;
// dof4 servo
int PIN_SERVO_DOF4 = 18;
// encoders
int PIN_ENC_DOF4 = 15;
int PIN_ENC_DOF5 = 22;
int PIN_ENC_DOF6 = 23;
// stepper 5 enable pin
int PIN_ENABLE_DOF5 = 26;
//
//// ----------- zero locations -------------------------
//// Zero variables
//double zero_DOF4 = 0;
//double zero_DOF5 = 0;
//double zero_DOF6 = 0;
//
//// ----------- state variables ------------------------
//double SPEED_4 = 0;
//double SPEED_5 = 0;
//double SPEED_6 = 0;
//
//// ---------- CAN frame IDs ---------------------------
//const int CAN_STATUS_ID    = 0x1F0;
//const int CAN_ENC_4_ID     = 0x1F1;
//const int CAN_ENC_5_ID     = 0x1F2;
//const int CAN_ENC_6_ID     = 0x1F3;


//-----------------
void setup() {
  Serial.begin(9600);
  pinMode(PIN_ENC_DOF4, INPUT);
  pinMode(PIN_ENABLE_DOF5, OUTPUT);
    
  pinMode(PIN_SERVO_DOF4, OUTPUT);

  digitalWrite(PIN_ENABLE_DOF5, HIGH);

  // configure servo
  servo4.attach(PIN_SERVO_DOF4);
}


unsigned long getPWM(int pin){
  unsigned long duration;
  duration = pulseIn(pin, HIGH);
  return duration;
}

double getAngleDOF4(){
  double zero = 0.0;
  double val = getPWM(PIN_ENC_DOF4)*360.0/4096.0;
  if(abs(val - zero) > 180.0){
    if(val > zero){
      return val - 360.0 - zero;
    }
    else{
      return val + 360.0 - zero;
    }
  }
  else {
    return val - zero;
  }
}

//------------------
void loop() {
  // send status frame CAN
  servo4.write(120);

}
