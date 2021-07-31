#include <Stepper.h>

// constants
const int STEPS_PER_REV = 200;

// pin declarations
int PIN_ENC_DOF4 = 34;
int PIN_ENC_DOF5 = 35;
int PIN_ENC_DOF6 = 32;
int PINS_MOT_DOF5[4] = {25, 26, 27, 14};
int PINS_MOT_DOF6[4] = {4, 16, 17, 5};

// motor objects
Stepper motor5(STEPS_PER_REV, 
  PINS_MOT_DOF5[0], 
  PINS_MOT_DOF5[1], 
  PINS_MOT_DOF5[2], 
  PINS_MOT_DOF5[3]
  );
Stepper motor6(STEPS_PER_REV, 
  PINS_MOT_DOF6[0], 
  PINS_MOT_DOF6[1], 
  PINS_MOT_DOF6[2], 
  PINS_MOT_DOF6[3]
  );

// Zero variables
long zero_DOF4 = 0;
long zero_DOF5 = 0;
long zero_DOF6 = 0;

void setup() {
  Serial.begin(9600);

  // encoder pins
  pinMode(PIN_ENC_DOF4, INPUT);
  pinMode(PIN_ENC_DOF5, INPUT);
  pinMode(PIN_ENC_DOF6, INPUT);

  // stepper motor pins
  for(int pin : PINS_MOT_DOF5) {
    pinMode(pin, OUTPUT);
  }
  for(int pin : PINS_MOT_DOF6) {
    pinMode(pin, OUTPUT);
  }
}

void loop() {
  if(Serial.available()){
    char input = Serial.read();
    if(input == '1'){
      setZeroAll();
      // motor5.setSpeed(20);
      // motor5.step(100);
    }

    if(input == '4'){
      Serial.println(getAngleDOF4());
    }
    if(input == '5'){
      Serial.println(getAngleDOF5());
    }
    if(input == '6'){
      Serial.println(getAngleDOF6());
    }
  }

}

long getPWM(int pin){
  unsigned long duration;
  duration = pulseIn(pin, HIGH);
  return duration;
}

long getAngleDOF4(){
  long zero = zero_DOF4;
  long val = getPWM(PIN_ENC_DOF4)*360.0/4096.0;
  if(abs(val - zero) > 180.0){
    if(val > zero){
      return val - (360.0 + zero);
    }
    else{
      return val + 360.0 - zero;
    }
  }
  else {
    return val - zero;
  }
}

long getAngleDOF5(){
  long zero = zero_DOF5;
  long val = getPWM(PIN_ENC_DOF5)*360.0/4096.0;
  if(abs(val - zero) > 90.0){
    if(val > zero){
       val - (360.0 + zero);
    }
    else{
       val + 360.0 - zero;
    }
  }
  else {
    return long(val - zero);
  }
}

long getAngleDOF6(){
  long zero = zero_DOF6;
  long val = getPWM(PIN_ENC_DOF6)*360.0/4096.0;
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

void runStepper(long speed, long rotations){

}