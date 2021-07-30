int ENC_four_pin = 34;
int ENC_five_pin = 35;
int ENC_six_pin = 32;

void setup() {
  Serial.begin(9600);
  pinMode(ENC_four_pin, INPUT);
  pinMode(ENC_five_pin, INPUT);
  pinMode(ENC_six_pin, INPUT);

}

void loop() {
  if(Serial.available()){
    char input = Serial.read();
    if(input == '4'){
      Serial.println(get_dof_four_angle());
    }
    if(input == '5'){
      Serial.println(get_dof_five_angle());
    }
    if(input == '6'){
      Serial.println(get_dof_six_angle());
    }
  }

}

int get_dof_four_angle(){
  unsigned long duration;
  duration = pulseIn(ENC_four_pin, HIGH);
  return duration;
}

int get_dof_five_angle(){
  unsigned long duration;
  duration = pulseIn(ENC_five_pin, HIGH);
  return duration;
}

int get_dof_six_angle(){
  unsigned long duration;
  duration = pulseIn(ENC_six_pin, HIGH);
  return duration;
}