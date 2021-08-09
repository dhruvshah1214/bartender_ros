#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <CAN.h>

Servo servo4;
AccelStepper stepper5(AccelStepper::DRIVER, 21, 19);
AccelStepper stepper6(AccelStepper::DRIVER, 33, 25);

// ------------- CONSTANTS -----------
// stepper const
#define  STEPPER_MAX_SPEED 6000
#define  Stepper_MIN_SPEED 0.1
// CAN const
#define  CAN_BITRATE 1000000

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

// ----------- zero locations -------------------------
// Zero variables
double zero_DOF4 = 0;
double zero_DOF5 = 0;
double zero_DOF6 = 0;

// ----------- state variables ------------------------
double SPEED_4 = 0;
double SPEED_5 = 0;
double SPEED_6 = 0;

// ---------- CAN frame IDs ---------------------------
const int CAN_STATUS_ID    = 0x1F0;
const int CAN_ENC_4_ID     = 0x1F1;
const int CAN_ENC_5_ID     = 0x1F2;
const int CAN_ENC_6_ID     = 0x1F3;


//-----------------
void setup() {
  Serial.begin(9600);
  Serial.println("can configured");

  // setup CAN
  CAN.setPins(PIN_CAN_RX, PIN_CAN_TX);
  if (!CAN.begin(CAN_BITRATE)) {
    while (1) {
      Serial.println("Starting CAN failed!");
    }
  }
  CAN.onReceive(onReceiveCAN);


  // configure servo
  servo4.attach(PIN_SERVO_DOF4);

  // configure steppers
  stepper5.setMaxSpeed(STEPPER_MAX_SPEED);
  stepper6.setMaxSpeed(STEPPER_MAX_SPEED);
  // enable pin stepper 5
  pinMode(PIN_ENABLE_DOF5, OUTPUT);
  digitalWrite(PIN_ENABLE_DOF5, HIGH);
  
  // encoder pins
  pinMode(PIN_ENC_DOF4, INPUT);
  pinMode(PIN_ENC_DOF5, INPUT);
  pinMode(PIN_ENC_DOF6, INPUT);

  // init encoder zeros
  zero_DOF4 = 0.0;
  zero_DOF5 = 254.71;
  zero_DOF6 = 15.12;
  
// init state
//  setSpeed_4(0.0);
//  setSpeed_5(0.0);
//  setSpeed_6(0.0);
}

//------------------
void loop() {
  // send status frame CAN

  // send dof4 angle
  float dof4Angle = (float) getAngleDOF4();
  Serial.println(dof4Angle);
  CAN.beginPacket(CAN_ENC_4_ID);
  int written4 = CAN.write((byte*) &dof4Angle, sizeof(dof4Angle));
  int success4 = CAN.endPacket();
//  Serial.printf("wrote %d bytes to CAN dof4 with success %d \n", written4, success4); 

  // send dof5 angle
  float dof5Angle = (float) getAngleDOF5();
//  Serial.println(dof5Angle);
  CAN.beginPacket(CAN_ENC_5_ID);
  int written5 = CAN.write((byte*) &dof5Angle, sizeof(dof5Angle));
  int success5 = CAN.endPacket();
//  Serial.printf("wrote %d bytes to CAN dof5 with success %d \n", written5, success5); 

  // send dof6 angle
  float dof6Angle = (float) getAngleDOF6();
//  Serial.println(dof6Angle);
  CAN.beginPacket(CAN_ENC_6_ID);
  int written6 = CAN.write((byte*) &dof6Angle, sizeof(dof6Angle));
  int success6 = CAN.endPacket();
//  Serial.printf("wrote %d bytes to CAN dof6 with success %d \n", written6, success6); 

  
//  MOT_4.write(SPEED_4);
//  MOT_6.setSpeed(SPEED_6);
//  MOT_5.setSpeed(SPEED_5);
//  MOT_6.runSpeed();
//  MOT_5.runSpeed();
//  if(Serial.available()){
//    char input = Serial.read();
//    if(input == '1'){
//      setSpeed_4(0.2);
//    }
//    else if(input == '2'){
//      setSpeed_4(-0.05);
//    }
//    //Encoder testing
//    if(input == '3'){
//      setZeroAll();
//    }
//    else if(input == '4'){
//      Serial.println(getAngleDOF4());
//    }
//    else if(input == '5'){
//      Serial.println(getAngleDOF5());
//    }
//    else if(input == '6'){
//      Serial.println(getAngleDOF6());
//    }
//    //Stepper Testing
//  }

}

void onReceiveCAN(int packetSize) {
  Serial.printf("received packet size %i \n", packetSize);
}

void setSpeed_4(double speed){
  SPEED_4 = (speed*90) + 90;
}

void setSpeed_5(double speed){
  SPEED_5 = speed*STEPPER_MAX_SPEED;
}

void setSpeed_6(double speed){
  SPEED_6 = speed*STEPPER_MAX_SPEED;
}

double getPWM(int pin){
  unsigned long duration;
  duration = pulseIn(pin, HIGH);
  return duration;
}

double getAngleDOF4(){
  double zero = zero_DOF4;
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

double getAngleDOF5(){
  double ratio = 20.0/36.0;
  double zero = zero_DOF5;
  double val = getPWM(PIN_ENC_DOF5)*360.0/4096.0;
  if(abs(val - zero) > 180.0){
    if(val > zero){
       return (val - 360.0 - zero)*ratio;
    }
    else{
       return (val + 360.0 - zero)*ratio;
    }
  }
  else {
    return (val - zero)*ratio;
  }
}

double getAngleDOF6(){
  double zero = zero_DOF6;
  double val = getPWM(PIN_ENC_DOF6)*360.0/4096.0;
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

void setZeroAll(){
  setZeroDOF4();
  setZeroDOF5();
  setZeroDOF6();
}

void setZeroDOF4(){
  zero_DOF4 = getPWM(PIN_ENC_DOF4)*360.0/4096.0;
}

void setZeroDOF5(){
  zero_DOF5 = getPWM(PIN_ENC_DOF5)*360.0/4096.0;
}

void setZeroDOF6(){
  zero_DOF6 = getPWM(PIN_ENC_DOF6)*360.0/4096.0;
}

void disableDOF5() {
  digitalWrite(PIN_ENABLE_DOF5, HIGH);
}

void enableDOF5() {
  digitalWrite(PIN_ENABLE_DOF5, LOW);
}
