#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <CAN.h>

Servo servo4;
AccelStepper stepper5(AccelStepper::DRIVER, 21, 19);
AccelStepper stepper6(AccelStepper::DRIVER, 33, 25);

// ------------- CONSTANTS -----------
// stepper const
#define STEPPER_MAX_SPEED 8000
#define Stepper_MIN_SPEED 0.1
// CAN const
#define CAN_BITRATE 1000000

// LOGGING - setting these flags to 1 will write the corresponding logs
#define LOG_ENCODER_4 0
#define LOG_ENCODER_5 0
#define LOG_ENCODER_6 1
#define LOG_CAN_WRITE_4 0
#define LOG_CAN_WRITE_5 0
#define LOG_CAN_WRITE_6 0
#define LOG_CAN_HAS_DATA 1
#define LOG_CAN_DATA 1
#define LOG_CAN_READ_4 0
#define LOG_CAN_READ_5 0
#define LOG_CAN_READ_6 1

#define LOG_ACTUAL_COMMANDED_SPEED_4 0
#define LOG_ACTUAL_COMMANDED_SPEED_5 0
#define LOG_ACTUAL_COMMANDED_SPEED_6 1


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
double ZERO_DOF4 = -149.15;
double ZERO_DOF5 = 210.23;
double ZERO_DOF6 = 59.24;

// ----------- state variables ------------------------
double targetPWM4 = 0;
double targetPWM5 = 0;
double targetPWM6 = 0;

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
  CAN.filter(0x1F0, 0x7F0);
//  CAN.onReceive(onReceiveCAN);

  // configure servo
  servo4.attach(PIN_SERVO_DOF4);

  // configure steppers
  stepper5.setMaxSpeed(STEPPER_MAX_SPEED);
  stepper6.setMaxSpeed(STEPPER_MAX_SPEED);
  // enable pin stepper 5
  pinMode(PIN_ENABLE_DOF5, OUTPUT);
  disableDOF5();
  
  // encoder pins
  pinMode(PIN_ENC_DOF4, INPUT);
  pinMode(PIN_ENC_DOF5, INPUT);
  pinMode(PIN_ENC_DOF6, INPUT);
}

//------------------
void loop() {
  // send status frame CAN

  // send dof4 angle
  float dof4Angle = (float) getAngleDOF4();
  if(LOG_ENCODER_4) {
    Serial.println(dof4Angle);
  }
  CAN.beginPacket(CAN_ENC_4_ID);
  int written4 = CAN.write((byte*) &dof4Angle, sizeof(dof4Angle));
  int success4 = CAN.endPacket();
  if(LOG_CAN_WRITE_4) {
    Serial.printf("wrote %d bytes to CAN dof4 with success %d \n", written4, success4); 
  }

  // send dof5 angle
  float dof5Angle = (float) getAngleDOF5();
  if(LOG_ENCODER_5) {
    Serial.println(dof5Angle);
  }
  CAN.beginPacket(CAN_ENC_5_ID);
  int written5 = CAN.write((byte*) &dof5Angle, sizeof(dof5Angle));
  int success5 = CAN.endPacket();
  if(LOG_CAN_WRITE_5) {
    Serial.printf("wrote %d bytes to CAN dof5 with success %d \n", written5, success5); 
  }
  
  // send dof6 angle
  float dof6Angle = (float) getAngleDOF6();
  if(LOG_ENCODER_6) {
    Serial.println(dof6Angle);
  }
  CAN.beginPacket(CAN_ENC_6_ID);
  int written6 = CAN.write((byte*) &dof6Angle, sizeof(dof6Angle));
  int success6 = CAN.endPacket();
  if(LOG_CAN_WRITE_6) {
    Serial.printf("wrote %d bytes to CAN dof6 with success %d \n", written6, success6); 
  }

  // read CAN bus for incoming msgs
  onReceiveCAN(CAN.parsePacket());

  // motion constraints
  double actuated4 = targetPWM4;
  double actuated5 = targetPWM5;
  double actuated6 = targetPWM6;

  if (dof4Angle > 175.0) {
    actuated4 = constrain(actuated4, -1.0, 0.0);
  } else if (dof4Angle < -175.0) {
    actuated4 = constrain(actuated4, 0.0, 1.0);
  }

  if (dof5Angle > 90) {
    actuated5 = constrain(actuated5, -1.0, 0.0);
  } else if (dof5Angle < -90.0) {
    actuated5 = constrain(actuated5, 0.0, 1.0);
  }

  if (dof6Angle > 175.0) {
    actuated6 = constrain(actuated6, -1.0, 0.0);
  } else if (dof5Angle < -175.0) {
    actuated6 = constrain(actuated6, 0.0, 1.0);
  }

  // write outputs to actuators
  if (LOG_ACTUAL_COMMANDED_SPEED_4) {
    Serial.printf("actuating dof 4: pwm speed %.02f\n", actuated4);
  }
  if (LOG_ACTUAL_COMMANDED_SPEED_5) {
    Serial.printf("actuating dof 5: pwm speed %.02f\n", actuated5);
  }
  if (LOG_ACTUAL_COMMANDED_SPEED_6) {
    Serial.printf("actuating dof 6: pwm speed %.02f\n", actuated6);
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
      float commandPWM4 = *((float*) canData);
      if(LOG_CAN_READ_4) {
        Serial.printf("received dof 4 speed command: %.04f\n", commandPWM4);
      }
      if (abs(commandPWM4) <= 1.0) {
        targetPWM4 = (double) commandPWM4;
      }
      else {
        Serial.printf("received out-of-range CAN command dof 4: %.02f\n", commandPWM4);
      }
      break;
    }
    case 5:
    {
      // command pwm speed dof 5
      float commandPWM5 = *((float*) canData);
      if(LOG_CAN_READ_5) {
        Serial.printf("received dof 5 speed command: %.04f\n", commandPWM5);
      }
      if (abs(commandPWM5) <= 1.0) {
        targetPWM5 = (double) commandPWM5;
      }
      else {
        Serial.printf("received out-of-range CAN command dof 5: %.02f\n", commandPWM5);
      }
      if (abs(commandPWM5) > 0.0) {
        enableDOF5();
      }
      break;
    }
    case 6:
    {
      // command pwm speed dof 6
      float commandPWM6 = *((float*) canData);
      if (abs(commandPWM6) <= 1.0) {
        targetPWM6 = (double) commandPWM6;
      }
      else {
        Serial.printf("received out-of-range CAN command dof 6: %.02f\n", commandPWM6);
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
  }
  delete [] canData;
}

// -----------------  setters // actuators  -------------------------

void run4(double pwmSpeed){
  int sp = (int) ((pwmSpeed*90.0) + 90.0);
  servo4.write(sp);
}

void run5(double pwmSpeed){
  double sp = pwmSpeed*STEPPER_MAX_SPEED;
  stepper5.setSpeed(sp);
  stepper5.runSpeed();
}

void run6(double pwmSpeed){
  double sp = pwmSpeed*STEPPER_MAX_SPEED;
  stepper6.setSpeed(sp);
  stepper6.runSpeed();
}

void disableDOF5() {
  digitalWrite(PIN_ENABLE_DOF5, HIGH);
}

void enableDOF5() {
  digitalWrite(PIN_ENABLE_DOF5, LOW);
}

// ---------------  getters // encoders -----------------

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
