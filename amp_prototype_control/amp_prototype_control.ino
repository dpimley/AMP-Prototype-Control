/*
 AMP prototype control

 Author - David Pimley
*/

#define pin_in_1 2  // digital out
#define pin_en_A 3  // PWM
#define pin_in_2 4  // digital out

/* 
   For Motor Control The Direction is Defined as:

   Setup:
   Red   --> OUT1 on L298N
   Black --> OUT2 on L298N

   Control:
   IN1 IN2 DIR
   H   L   CW
   L   H   CCW
   
*/

void setup() {
  pinMode(pin_en_A, OUTPUT);
  pinMode(pin_in_1, OUTPUT);
  pinMode(pin_in_2, OUTPUT);
}

void loop() {
  // Test Single Direction Spin @ Half Speed
  digitalWrite(pin_in_1, HIGH);
  digitalWrite(pin_in_2, LOW);
  analogWrite(pin_en_A, 155);
  delay(5000);
  analogWrite(pin_en_A, 255);
   digitalWrite(pin_in_1, LOW);
  digitalWrite(pin_in_2, HIGH);
  delay(5000);
}
