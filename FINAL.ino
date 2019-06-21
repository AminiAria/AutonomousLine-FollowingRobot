#include <Servo.h>
int ENA = 5; // ENA (PWM or Enable for Motor A)
int ENB = 11; // ENB (PWM or Enable for Motor B)
int motorA1 = 6; // IN A1 or IN1 (Motor A Direction)
int motorA2 = 7; // IN A2 or IN2 (Motor A Direction)
int motorB1 = 9; // IN B1 or IN3 (Motor B Direction)
int motorB2 = 8; // IN B2 or IN4 (Motor B Direction)

const int C4 = 262 ; // frequency in Hz 
const int G4 = 392; 
const int F4 = 349;
const int E4 = 330;
const int D4 = 294;
const int C5 = 523;
const int qNote = 500; //quarter note time is 500ms


const int slightRightA = 100;
const int slightRightB = 200;
const int hardRightA = 120;
const int hardRightB = 100;
const int slightLeftA = 200;
const int slightLeftB = 100;
const int hardLeftA = 120;
const int hardLeftB = 100;
const int straight = 100;

int lastPosition;
int leftSensor = 10;
int centerSensor = 4;
int rightSensor = 2;
 

int servoPin = 3;
Servo Servo1;
// Pins used for digital input of obstacle sensors
const int RangeTriggerPin = A5; // Rangefinder Trigger input pin
const int RangeEchoPin = A4; // Rangefinder Echo Sensor output pin
const unsigned long RangeTimeout = 4000; // usec timeout value.
// Declare variables
unsigned long EchoDelay = 0;

//Return Distance value & Print Distance Value//
float Distance = 0; // create a variable called Distance
boolean Yikes;



void slightRight(){
  analogWrite(ENA,slightRightA);
  analogWrite(ENB,slightRightB);
  digitalWrite(motorA1,HIGH);
  digitalWrite(motorA2,LOW);
  digitalWrite(motorB1,HIGH);
  digitalWrite(motorB2,LOW);
}
void hardRight(){
  analogWrite(ENA,hardRightA);
  analogWrite(ENB,hardRightB);
  //digitalWrite(motorA1,HIGH);
  //digitalWrite(motorA2,LOW);
  digitalWrite(motorA1,LOW);
  digitalWrite(motorA2,HIGH);
  digitalWrite(motorB1,HIGH);
  digitalWrite(motorB2,LOW);
}
void slightLeft(){
  analogWrite(ENA,slightLeftA);
  analogWrite(ENB,slightLeftB);
  digitalWrite(motorA1,HIGH);
  digitalWrite(motorA2,LOW);
  digitalWrite(motorB1,HIGH);
  digitalWrite(motorB2,LOW);
}
void hardLeft(){
  analogWrite(ENA,hardLeftA);
  analogWrite(ENB,hardLeftB);
  digitalWrite(motorA1,HIGH);
  digitalWrite(motorA2,LOW);
  //digitalWrite(motorB1,HIGH);
  //digitalWrite(motorB2,LOW);
  digitalWrite(motorB1,LOW);
  digitalWrite(motorB2,HIGH);
}
void forward(){
  analogWrite(ENA,straight);
  analogWrite(ENB,straight);
  digitalWrite(motorA1,HIGH);
  digitalWrite(motorA2,LOW);
  digitalWrite(motorB1,HIGH);
  digitalWrite(motorB2,LOW);
}
void reverse(){
  analogWrite(ENA,straight);
  analogWrite(ENB,straight);
  digitalWrite(motorA1,LOW);
  digitalWrite(motorA2,HIGH);
  digitalWrite(motorB1,LOW);
  digitalWrite(motorB2,HIGH);
}
void stop(){
  digitalWrite(ENA,LOW);
  digitalWrite(ENB,LOW);
}
void followLine(){
  int leftValue = digitalRead(leftSensor);
  int centerValue = digitalRead(centerSensor);
  int rightValue = digitalRead(rightSensor);
  if (centerValue == HIGH){ // If at least partially on‐track ‐  Sensors: [x B x]
    // Determine if we are slightly off track in either direction
    if (leftValue == HIGH && rightValue == LOW){ // If slightly off track toward the right – Sensors: [B B W]
      lastPosition = 1; //off‐track Right 
      slightLeft();
      //Serial.println("slightLeft");
    }
    if (rightValue == HIGH && leftValue == LOW){ // If slightly off track toward the left  [W B B]
      lastPosition = 2; //off‐track Left
      slightRight();
      //Serial.println("slightRight");
      }
    if (leftValue == LOW && rightValue == LOW){ // If completely on track  [W B W]
       lastPosition = 0; //on‐track
       forward();   
       //Serial.println("forward");     
      }
    else if (centerValue == HIGH && rightValue == HIGH && leftValue == HIGH){
      stop();
      eggReadyTone();
      Servo1.write(172);  
    }
  }
  else if (centerValue == LOW){ //If off track - Sensors: [x W x]
    //Determine which direction we are off
    if (leftValue == HIGH && rightValue == LOW){ //if off track toward the right - Sensors: [B W W]
      lastPosition = 1; //off‐track Right
      hardLeft();
      //Serial.println("hardtLeft");
    } 
    if (rightValue == HIGH && leftValue == LOW){ //if off track toward the left [W W B]
        lastPosition = 2; //off-track left
        hardRight();
        //Serial.println("hardRight");
    }  
    if (rightValue == LOW && leftValue == LOW){ //if completely off track - Sensors: [W W W]
      switch(lastPosition){
        case(1): hardLeft(); /*Serial.println("case(1)");*/ break; //last turn was left
        case(2): hardRight(); /*Serial.println("case(2)");*/ break; //last turn was right
        case(0): reverse(); /*Serial.println("case(0)");*/ break; //last time was moving straight
      }
    }
  }
}
float ObstacleDistance (float WithinInches){
  float x;
  // Check Range Sensor by pulsing the trigger
  digitalWrite(RangeTriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(RangeTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(RangeTriggerPin, LOW);
  // The other pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  EchoDelay = pulseIn(RangeEchoPin, HIGH, RangeTimeout);
  Distance = (EchoDelay / 74.0) / 2.0; // Convert time to inches one-way
  // Speed of sound is 0.0135 inches/ microsecond
  // or 74 microseconds per inch.
  if (EchoDelay < 300) { //makes Distance 0 if EchoDelay is less than 300
    Distance = 0;
  }
  if (Distance < WithinInches) {
    x = Distance;
  }
  else {
    x = 0;
  }
  //Serial.print("EchoDelay = "); Serial.println(EchoDelay);
  //Serial.print("Distance = "); Serial.println(x);
  return x;
}
boolean ObstacleDetected(float WithinInches) {  //Detect Obstacle's Existence
  if (ObstacleDistance(WithinInches) > 0) {
    return true;
  }
  else {
    return false;
  }
}
void avoidObstacles(){
  int leftValue = digitalRead(leftSensor);
  int centerValue = digitalRead(centerSensor);
  int rightValue = digitalRead(rightSensor);
  Yikes = ObstacleDetected(5);
  if (Yikes == true){
    if (rightValue == LOW || leftValue == LOW){
      stop();
      objectSiren();
      noTone(A3);  
    } 
  }  
  else{
    followLine();
  }
}
void flashLED(){
  digitalWrite(13,HIGH);
  delay(300);
  digitalWrite(13,LOW);
  delay(300);
}
void siren(){ //makes a siren for 5 seconds
  for (int x = 0; x < 5; x++){
    tone(A3,400);
    delay(500);
    tone(A3,300);
    delay(500);
    noTone(A3);
  }
}
void objectSiren(){
  for (int x = 500; x < 1400; x++){ //initializes a variable x at 400, and adds one to x until it is 1400
    tone(A3,x); //plays a tone at a frequency of x
    delay(3); //plays each tone for 3ms
  }
  for (int x = 1400; x > 500; x--){ //initializes x at 1400 and subtracts one from x until it is 400
    tone(A3,x); //plays a tone at a frequency of x
    delay(3); //plays each tone for 3ms
  }
}
void eggReadyTone(){
  tone (A3, C4, 2 * qNote); //plays C4 for a half note
  delay(2 * qNote);
  tone(A3, G4, 4 * qNote); //plays G4 for a whole note
  delay(4 * qNote);
  tone(A3, F4, qNote); //plays F4 for a quarter note
  delay(qNote);
  tone(A3, E4, .5 * qNote); //plays E4 for an eighth note
  delay(.5 * qNote);
  tone(A3, D4, .5*qNote); //plays D4 for an eighth note
  delay(.5 * qNote);
  tone(A3, C5, 2 * qNote); //plays C5 for a half note
  delay(2 * qNote);
  tone(A3, G4, 4 * qNote); //plays G4 for a whole note
  delay(4 * qNote);
  noTone(A3);
}
 
void setup() {
  pinMode(13,OUTPUT);//pin 13 LED
  pinMode(A3, OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(RangeTriggerPin, OUTPUT);
  pinMode(RangeEchoPin, INPUT);
  pinMode(leftSensor,INPUT);
  pinMode(centerSensor,INPUT);
  pinMode(rightSensor,INPUT);
  Serial.begin(9600); //prints some text to serial monitor, baud rate is 9600
  Servo1.attach(servoPin);
  Servo1.write(95);//sets servo to forward
  //siren();
}

void loop() {
  avoidObstacles();
}

