#include <AccelStepper.h>
#include <ESP32Servo.h>

AccelStepper MOT_5(AccelStepper::DRIVER, 12, 13);
AccelStepper MOT_6(AccelStepper::DRIVER, 27, 14);
Servo MOT_4;

//Stepper Maxes
#define  Stepper_MAX_SPEED 4000
#define  Stepper_MIN_SPEED 0.1

// pin declarations
int PIN_ENC_DOF4 = 25;
int PIN_ENC_DOF5 = 32;
int PIN_ENC_DOF6 = 33;

// Zero variables
double zero_DOF4 = 0;
double zero_DOF5 = 0;
double zero_DOF6 = 0;

double SPEED_4 = 0;
double SPEED_5 = 0;
double SPEED_6 = 0;

//-----------------
void setup() {
  Serial.begin(9600);

  MOT_4.attach(18);
  MOT_5.setMaxSpeed(4000.0);
  MOT_6.setMaxSpeed(4000.0);
  // encoder pins
  pinMode(PIN_ENC_DOF4, INPUT);
  pinMode(PIN_ENC_DOF5, INPUT);
  pinMode(PIN_ENC_DOF6, INPUT);

  setSpeed_4(0.0);
  setSpeed_5(0.0);
  setSpeed_6(0.0);
}

//------------------
void loop() {
  MOT_4.write(SPEED_4);
  MOT_6.setSpeed(SPEED_6);
  MOT_5.setSpeed(SPEED_5);
  MOT_6.runSpeed();
  MOT_5.runSpeed();
  if(Serial.available()){
    char input = Serial.read();
    if(input == '1'){
      setSpeed_4(0.2);
    }
    else if(input == '2'){
      setSpeed_4(-0.05);
    }
    //Encoder testing
    if(input == '3'){
      setZeroAll();
    }
    else if(input == '4'){
      Serial.println(getAngleDOF4());
    }
    else if(input == '5'){
      Serial.println(getAngleDOF5());
    }
    else if(input == '6'){
      Serial.println(getAngleDOF6());
    }
    //Stepper Testing
  }

}

void setSpeed_4(double speed){
  SPEED_4 = (speed*90) + 90;
}

void setSpeed_5(double speed){
  SPEED_5 = speed*Stepper_MAX_SPEED;
}

void setSpeed_6(double speed){
  SPEED_6 = speed*Stepper_MAX_SPEED;
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