#include <CAN.h>
#include <FastAccelStepper.h>
#include "ESP32_ISR_Servo.h"

int servoIndex4;
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper5 = NULL;
FastAccelStepper *stepper6 = NULL;

//AccelStepper stepper5(AccelStepper::DRIVER, 21, 19);
//AccelStepper stepper6(AccelStepper::DRIVER, 33, 25);

// ------------- CONSTANTS -----------
// stepper const
#define STEPPER_MAX_SPEED 6000 // RPM
#define Stepper_MIN_SPEED 0.1
#define RPM_TO_MICROS_PER_STEP(RPM) 300000.0/RPM // RPM^-1 min/rev * 1/200 rev/step * 60 sec/min * 1000000 micros/sec = RPM^-1 min/rev * 300000 rev-micros/min-step
#define STEPS_PER_REV 200

// CAN const
#define CAN_BITRATE 1000000

// LOGGING - setting these flags to 1 will write the corresponding logs
#define LOG_ENCODER_4 0
#define LOG_ENCODER_5 0
#define LOG_ENCODER_6 0
#define LOG_CAN_WRITE_4 0
#define LOG_CAN_WRITE_5 0
#define LOG_CAN_WRITE_6 0
#define LOG_CAN_HAS_DATA 1
#define LOG_CAN_DATA 1
#define LOG_CAN_READ_4 0
#define LOG_CAN_READ_5 0
#define LOG_CAN_READ_6 0

#define LOG_ACTUAL_COMMANDED_DUTYCYCLE_4 0
#define LOG_ACTUAL_COMMANDED_DUTYCYCLE_5 1
#define LOG_ACTUAL_COMMANDED_DUTYCYCLE_6 0
#define LOG_ACTUAL_COMMANDED_SPEED_4 0
#define LOG_ACTUAL_COMMANDED_SPEED_5 1
#define LOG_ACTUAL_COMMANDED_SPEED_6 0


// ----------- pin declarations ---------------------
// CAN
const int PIN_CAN_TX = 17;
const int PIN_CAN_RX = 16;
// dof4 servo
int PIN_SERVO_DOF4 = 18;
// steppers
int PIN_STEP_DOF5 = 21;
int PIN_DIR_DOF5 = 19;
int PIN_ENABLE_DOF5 = 26;
int PIN_STEP_DOF6 = 33;
int PIN_DIR_DOF6 = 25;
// encoders
int PIN_ENC_DOF4 = 15;
int PIN_ENC_DOF5 = 22;
int PIN_ENC_DOF6 = 23;
// zero btn
int PIN_ZERO_BTN = 23;


// ----------- zero locations -------------------------
// Zero variables
double ZERO_DOF4 = -149.15;
double ZERO_DOF5 = 210.23;
double ZERO_DOF6 = 59.24;

// ----------- state variables ------------------------
double targetAngle4 = 0;
double targetAngle5 = 0;
double targetAngle6 = 0;

// ---------- CAN frame IDs ---------------------------
const int CAN_STATUS_ID      = 0x1F0;
const int CAN_ENC_4_ID       = 0x1F1;
const int CAN_ENC_5_ID       = 0x1F2;
const int CAN_ENC_6_ID       = 0x1F3;
const int CAN_SET_4_ID       = 0x1F4;
const int CAN_SET_5_ID       = 0x1F5;
const int CAN_SET_6_ID       = 0x1F6;
const int CAN_ENABLE_5_ID    = 0x1F7;
const int CAN_ENABLE_6_ID    = 0x1F8;
const int CAN_DISABLE_ID     = 0x1F9;
const int CAN_ZERO_ID        = 0x1FA;


//------------ SETUP ---------------------------------------

void setup() {
  Serial.begin(57600);

  // setup CAN
  CAN.setPins(PIN_CAN_RX, PIN_CAN_TX);
  if (!CAN.begin(CAN_BITRATE)) {
    while (1) {
      Serial.println("Starting CAN failed!");
    }
  }
  CAN.filter(0x1F0, 0x7F0);
  Serial.println("can configured");
//  CAN.onReceive(onReceiveCAN);

  // configure servo
  ESP32_ISR_Servos.useTimer(2);
  targetAngle4 = 0.0;
  servoIndex4 = ESP32_ISR_Servos.setupServo(PIN_SERVO_DOF4);
  run4(targetAngle4);


  // configure steppers
  engine.init();
  stepper5 = engine.stepperConnectToPin(PIN_STEP_DOF5);
  if (stepper5) {
      stepper5->setDirectionPin(PIN_DIR_DOF5);
      stepper5->setEnablePin(PIN_ENABLE_DOF5);
      stepper5->setAutoEnable(true);
      stepper5->setSpeedInUs(RPM_TO_MICROS_PER_STEP(0.3 * STEPPER_MAX_SPEED));
      stepper5->setAcceleration(500000);    // 500,000 steps/s²
      stepper5->enableOutputs();
      stepper5->stopMove();
      stepper5->setCurrentPosition(0);
  }
  else {
    while (1) {
      Serial.println("setting up stepper 5 failed!");
    }
  }
  stepper6 = engine.stepperConnectToPin(PIN_STEP_DOF6);
  if (stepper6) {
      stepper6->setDirectionPin(PIN_DIR_DOF6);
      stepper6->setSpeedInUs(RPM_TO_MICROS_PER_STEP(0.3 * STEPPER_MAX_SPEED));
      stepper6->setAcceleration(500000);    // 500,000 steps/s²
      stepper6->move(100);
  }
  else {
    while (1) {
      Serial.println("setting up stepper 6 failed!");
    }
  }
  
  // encoder pins
  pinMode(PIN_ENC_DOF4, INPUT);
  pinMode(PIN_ENC_DOF5, INPUT);
  pinMode(PIN_ENC_DOF6, INPUT);

  // zero button
  pinMode(PIN_ZERO_BTN, INPUT_PULLDOWN);
}



//------------------ LOOP -------------------------------



void loop() {
  // send status frame CAN
  
//  // send dof4 angle
//  float dof4Angle = (float) getAngleDOF4();
//  if(LOG_ENCODER_4) {
//    Serial.println(dof4Angle);
//  }
//  CAN.beginPacket(CAN_ENC_4_ID);
//  int written4 = CAN.write((byte*) &dof4Angle, sizeof(dof4Angle));
//  int success4 = CAN.endPacket();
//  if(LOG_CAN_WRITE_4) {
//    Serial.printf("wrote %d bytes to CAN dof4 with success %d \n", written4, success4); 
//  }
//
//  // send dof5 angle
//  float dof5Angle = (float) getAngleDOF5();
//  if(LOG_ENCODER_5) {
//    Serial.println(dof5Angle);
//  }
//  CAN.beginPacket(CAN_ENC_5_ID);
//  int written5 = CAN.write((byte*) &dof5Angle, sizeof(dof5Angle));
//  int success5 = CAN.endPacket();
//  if(LOG_CAN_WRITE_5) {
//    Serial.printf("wrote %d bytes to CAN dof5 with success %d \n", written5, success5); 
//  }
//  
//  // send dof6 angle
//  float dof6Angle = (float) getAngleDOF6();
//  if(LOG_ENCODER_6) {
//    Serial.println(dof6Angle);
//  }
//  CAN.beginPacket(CAN_ENC_6_ID);
//  int written6 = CAN.write((byte*) &dof6Angle, sizeof(dof6Angle));
//  int success6 = CAN.endPacket();
//  if(LOG_CAN_WRITE_6) {
//    Serial.printf("wrote %d bytes to CAN dof6 with success %d \n", written6, success6); 
//  }

  // read CAN bus for incoming msgs
  onReceiveCAN(CAN.parsePacket());

  // motion constraints
  double actuated4 = targetAngle4;
  double actuated5 = targetAngle5;
  double actuated6 = targetAngle6;

//  if (dof4Angle > 175.0) {
//    actuated4 = constrain(actuated4, -1.0, 0.0);
//  } else if (dof4Angle < -175.0) {
//    actuated4 = constrain(actuated4, 0.0, 1.0);
//  }
//
//  if (dof5Angle > 90) {
//    actuated5 = constrain(actuated5, -1.0, 0.0);
//  } else if (dof5Angle < -90.0) {
//    actuated5 = constrain(actuated5, 0.0, 1.0);
//  }
//
//  if (dof6Angle > 175.0) {
//    actuated6 = constrain(actuated6, -1.0, 0.0);
//  } else if (dof5Angle < -175.0) {
//    actuated6 = constrain(actuated6, 0.0, 1.0);
//  }

  // write outputs to actuators
  if (LOG_ACTUAL_COMMANDED_DUTYCYCLE_4) {
    Serial.printf("actuating dof 4: command %.02f\n", actuated4);
  }
  if (LOG_ACTUAL_COMMANDED_DUTYCYCLE_5) {
    Serial.printf("actuating dof 5: command %.02f\n", actuated5);
  }
  if (LOG_ACTUAL_COMMANDED_DUTYCYCLE_6) {
    Serial.printf("actuating dof 6: command %.02f\n", actuated6);
  }

  run4(actuated4);
  run5(actuated5);
  run6(actuated6);
}

void onReceiveCAN(int packetSize) {
  if(packetSize == 0) {
    return;
  }
  if(LOG_CAN_HAS_DATA) {
    Serial.printf("received packet size %d \n", packetSize);
  }
    
  long packetID = CAN.packetId();
  int command = packetID & 0xF;
  
  int availableBytes = CAN.available();
  if(LOG_CAN_HAS_DATA) {
    Serial.printf("CAN -- ID is %04x, command is %02x\n", packetID, command);
    Serial.printf("CAN -- %d bytes available \n", availableBytes);
  }
    
  byte *canData = new byte[availableBytes];
  CAN.readBytes(canData, availableBytes);

  if(LOG_CAN_DATA) {
    Serial.printf("CAN -- data: %d -- ", sizeof(canData));
    for(int i = 0; i < availableBytes; i++) {
      byte b = *(canData + i);
      Serial.printf("%02X", b);
    }
    Serial.printf("\n");
  }
  
  switch(command) {
    case 0:
      break;
    case 4:
    {
      // command pwm speed dof 4
      float command4 = *((float*) canData);
      targetAngle4 = constrain((double) command4, -90.0, 90.0);
      if(LOG_CAN_READ_4) {
        Serial.printf("received dof 4 pos command: %.04f\n", targetAngle4);
      }
      break;
    }
    case 5:
    {
      // command pwm speed dof 5
      float command5 = *((float*) canData);
      targetAngle5 = constrain((double) command5, -90.0, 90.0);
      if(LOG_CAN_READ_5) {
        Serial.printf("received dof 5 pos command: %.04f\n", targetAngle5);
      }
      break;
    }
    case 6:
    {
      // command pwm speed dof 6
      float command6 = *((float*) canData);
      targetAngle6 = constrain((double) command6, -180.0, 180.0);
      if(LOG_CAN_READ_6) {
        Serial.printf("received dof 6 pos command: %.04f\n", targetAngle6);
      }
      break;
    }
    case 7:
      // enable stepper dof 5
      break;
    case 8:
      // enable stepper dof 6
      break;
    case 9:
      // disable / neutralize robot
      break;
    case 10:
      // zero positions
      Serial.printf("received zero command\n");
      zero();
      break;
  }
  delete [] canData;
}

// -----------------  setters // actuators  -------------------------

void run4(double targetAngle){
  int sp = 45 + (int) (targetAngle/2.0);
  ESP32_ISR_Servos.setPosition(servoIndex4, sp);
  if(LOG_ACTUAL_COMMANDED_SPEED_4) {
    Serial.printf("running servo 4 @@@ %.02f!!!\n", sp);
  }
}

void run5(double targetAngle){
  stepper5->move(targetAngle * STEPS_PER_REV / 360.0);
  stepper5->runForward();
  if(LOG_ACTUAL_COMMANDED_SPEED_5) {
    Serial.printf("running stepper 5 @@@ %.02f!!!\n", stepper5->getCurrentPosition());
  }
}

void run6(double targetAngle){
  stepper6->moveTo(targetAngle * STEPS_PER_REV / 360.0);
}

void disableDOF5() {
  digitalWrite(PIN_ENABLE_DOF5, HIGH);
}

void enableDOF5() {
  digitalWrite(PIN_ENABLE_DOF5, LOW);
}

// ---------------  getters // encoders // zero -----------------

double getAngleDOF4(){
  double val = getRawAngle(PIN_ENC_DOF4);
  return getNeg180To180(val, ZERO_DOF4);
}

double getAngleDOF5(){
  double DOF5_ENC_SHAFT_GEARING = 20.0/36.0;
  double val = getGearedAngle(PIN_ENC_DOF5, DOF5_ENC_SHAFT_GEARING);
  return getGearedPlusMinusAngle(val, ZERO_DOF5, DOF5_ENC_SHAFT_GEARING);
}

double getAngleDOF6(){
  double val = getRawAngle(PIN_ENC_DOF6);
  return getNeg180To180(val, ZERO_DOF6);
}

void zero() {
  stepper5->setCurrentPosition(0);
  stepper6->setCurrentPosition(0);
}

// -------------------   utils   ------------------------

double getPWM(int pin){
  unsigned long duration;
  duration = pulseIn(pin, HIGH);
  return duration;
}

double getRawAngle(int pin) {
   double val = getPWM(pin)*360.0/4096.0;
   return val;
}

double getGearedAngle(int pin, double ratio) {
   double val = getPWM(pin)*360.0/4096.0*ratio;
   return val;
}

double getNeg180To180(double rawAngle, double zero) {
  if(abs(rawAngle - zero) > 180.0){
    if(rawAngle > zero){
      return rawAngle - 360.0 - zero;
    }
    else{
      return rawAngle + 360.0 - zero;
    }
  }
  else {
    return rawAngle - zero;
  }
}

double getGearedPlusMinusAngle(double gearedAngle, double zero, double ratio) {
  double baseline = 360.0 * ratio;
  zero = zero * ratio;
  if(abs(gearedAngle - zero) > baseline/2.0){
    if(gearedAngle > zero){
      return gearedAngle - baseline - zero;
    }
    else{
      return gearedAngle + baseline - zero;
    }
  }
  else {
    return gearedAngle - zero;
  }
}
