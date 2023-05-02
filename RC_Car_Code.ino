#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>
#define L_S A0 //ir sensor Left
#define R_S A1 //ir sensor Right
int Speed = 150;
#define echo A3    //Echo pin
#define trigger A2 //Trigger pin
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);
int Set=15;
int distance_L, distance_F, distance_R; 
char value;
Servo servoLook; 
byte maxDist = 150;                               //Maximum sensing distance 
(Objects further than this distance are ignored)
byte stopDist = 50;                               //Minimum distance from an 
object to stop in cm
float timeOut = 2*(maxDist+10)/100/340*1000000;   //Maximum time to wait for a 
return signal
int motorOffset = 10;                             //Factor to account for one 
side being more powerful
int turnSpeed = 50;   
void setup(){
Serial.begin(9600);
  motor1.setSpeed(Speed);
  motor2.setSpeed(Speed);
  motor3.setSpeed(Speed);
  motor4.setSpeed(Speed);
  
pinMode(R_S, INPUT); // declare if sensor as input  
pinMode(L_S, INPUT); // declare ir sensor as input
pinMode(echo, INPUT );// declare ultrasonic sensor Echo pin as input
pinMode(trigger, OUTPUT); // declare ultrasonic sensor Trigger pin as 
Output  
servoLook.attach(10);
pinMode(10, OUTPUT);
 
distance_F = Ultrasonic_read();
}
void loop(){
value = Serial.read();
switch (value){
//front light = W // Back light = U // Horn = V
case 'W':
while(1){
 servoLook.write(90);                        
//Set the servo to look straight ahead
  delay(750);
  int distance = getDistance();                   //Check that there are no 
objects ahead
  if(distance >= stopDist)                        //If there are no objects 
within the stopping distance, move forward
  {
    moveForward();
  }
  while(distance >= stopDist)                     //Keep checking the object 
distance until it is within the minimum stopping distance
  {
    distance = getDistance();
    delay(250);
  }
  stopMove();                                     //Stop the motors
  int turnDir = checkDirection();                 //Check the left and right 
object distances and get the turning instruction
  Serial.print(turnDir);
  switch (turnDir)                                //Turn left, turn around or 
turn right depending on the instruction
  {
    case 0:                                       //Turn left
      turnLeftOBS (400);
      break;
    case 1:                                       //Turn around
      turnLeftOBS (700);
      break;
    case 2:                                       //Turn right
      turnRightOBS (400);
      break;
  }
}
case 'U':
      Speed=100;
       motor1.setSpeed(Speed);
       motor2.setSpeed(Speed);
      motor3.setSpeed(Speed);
        motor4.setSpeed(Speed);
while(1){
distance_F = Ultrasonic_read();
Serial.print("D F=");Serial.println(distance_F);
//if Right Sensor and Left Sensor are at White color then it will call forword 
function
 if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 0)){
  if(distance_F > Set){forword();}
                  else{Check_side();}  
 }  
 
//if Right Sensor is Black and Left Sensor is White then it will call turn Right
function
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 0)){turnRight();}  
//if Right Sensor is White and Left Sensor is Black then it will call turn Left 
function
else if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 1)){turnLeft();} 
    
delay(10);
}
case 'V':
while(1){
if (Serial.available() > 0) {
    value = Serial.read();
  }
  if (value == 'F') {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
  } else if (value == 'B') {
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
  } else if (value == 'R') {
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
  } else if (value == 'L') {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
  } else {
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
  }
}
}
}
void accelerate()                                 //Function to accelerate the 
motors from 0 to full speed
{
  for (int i=0; i<Speed; i++)                //Loop from 0 to full speed
  {
    motor1.setSpeed(i);                        //Set the motors to the current 
loop speed
    motor2.setSpeed(i);
    motor3.setSpeed(i+motorOffset);
    motor4.setSpeed(i+motorOffset);
    delay(10);
  }
}
void decelerate()                                 //Function to decelerate the 
motors from full speed to zero
{
  for (int i=Speed; i!=0; i--)               //Loop from full speed to 0
  {
    motor1.setSpeed(i);                        //Set the motors to the current 
loop speed
    motor2.setSpeed(i);
    motor3.setSpeed(i+motorOffset);
    motor4.setSpeed(i+motorOffset); 
    delay(10);
  }
}
void servoPulse (int pin, int angle){
int pwm = (angle*11) + 500;      // Convert angle to microseconds
 digitalWrite(pin, HIGH);
 delayMicroseconds(pwm);
 digitalWrite(pin, LOW);
 delay(50); // Refresh cycle of servo
}
//**********************Ultrasonic_read****************************
long Ultrasonic_read(){
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  long time = pulseIn (echo, HIGH);
  return time / 29 / 2;
}
void compareDistance(){
   if(distance_L > distance_R){
  turnLeft();
  delay(1000);
  
  forword();
  delay(1200);
  
  turnRight();
  delay(2000);
  
  forword();
  delay(1200);
  
  turnRight();
  delay(1000);
  }
  
  else{
 turnRight();
  delay(1000);
 
 forword();
  delay(1200);
 
 turnLeft();
  delay(2000);
  
  forword();
  delay(1200);  
  
  turnLeft();
  delay(1000);
  }
}
void Check_side(){
    Stop();
    delay(100);
 for (int angle = 70; angle <= 140; angle += 5)  {
   servoPulse(10, angle);  }
    delay(300);
    distance_R = Ultrasonic_read();
    Serial.print("D R=");Serial.println(distance_R);
    delay(100);
  for (int angle = 140; angle >= 0; angle -= 5)  {
   servoPulse(10, angle);  }
    delay(500);
    distance_L = Ultrasonic_read();
    Serial.print("D L=");Serial.println(distance_L);
    delay(100);
 for (int angle = 0; angle <= 70; angle += 5)  {
   servoPulse(10, angle);  }
    delay(300);
    compareDistance();
}
void forword(){  //forword
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
}
void backword(){ //backword
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
}
void turnRight(){ //turnRight
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
}
void turnLeft(){ //turnLeft
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
}
void Stop(){ //stop
 motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
}
void moveForward()                                //Set all motors to run 
forward
{
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}
void stopMove()                                   //Set all motors to stop
{
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
void turnLeftOBS(int duration)                                 //Set motors to 
turn left for the specified duration then stop
{
  motor1.setSpeed(Speed+turnSpeed);                 //Set the motors to the 
motor speed
  motor2.setSpeed(Speed+turnSpeed);
  motor3.setSpeed(Speed+motorOffset+turnSpeed);
  motor4.setSpeed(Speed+motorOffset+turnSpeed);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(duration);
  motor1.setSpeed(Speed);                           //Set the motors to the 
motor speed
  motor2.setSpeed(Speed);
  motor3.setSpeed(Speed+motorOffset);
  motor4.setSpeed(Speed+motorOffset);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  
}
void turnRightOBS(int duration)                                //Set motors to 
turn right for the specified duration then stop
{
  motor1.setSpeed(Speed+turnSpeed);                 //Set the motors to the 
motor speed
  motor2.setSpeed(Speed+turnSpeed);
  motor3.setSpeed(Speed+motorOffset+turnSpeed);
  motor4.setSpeed(Speed+motorOffset+turnSpeed);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(duration);
  motor1.setSpeed(Speed);                           //Set the motors to the 
motor speed
  motor2.setSpeed(Speed);
  motor3.setSpeed(Speed+motorOffset);
  motor4.setSpeed(Speed+motorOffset);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
int getDistance()                                   //Measure the distance to an
object
{
  unsigned long pulseTime;                          //Create a variable to store
the pulse travel time
  int distance;                                     //Create a variable to store
the calculated distance
  digitalWrite(trigger, HIGH);                         //Generate a 10 
microsecond pulse
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  pulseTime = pulseIn(echo, HIGH, timeOut);         //Measure the time for the 
pulse to return
  distance = (float)pulseTime * 340 / 2 / 10000;    //Calculate the object 
distance based on the pulse time
  return distance;
}
int checkDirection()                                            //Check the left
and right directions and decide which way to turn
{
  int distances [2] = {0,0};                                    //Left and right
distances
  int turnDir = 1;                                              //Direction to 
turn, 0 left, 1 reverse, 2 right
  servoLook.write(180);                                         //Turn servo to 
look left
  delay(500);
  distances [0] = getDistance();                                //Get the left 
object distance
  servoLook.write(0);                                           //Turn servo to 
look right
  delay(1000);
  distances [1] = getDistance();                                //Get the right 
object distance
  if (distances[0]>=200 && distances[1]>=200)                   //If both 
directions are clear, turn left
    turnDir = 0;
  else if (distances[0]<=stopDist && distances[1]<=stopDist)    //If both 
directions are blocked, turn around
    turnDir = 1;
  else if (distances[0]>=distances[1])                          //If left has 
more space, turn left
    turnDir = 0;
  else if (distances[0]<distances[1])                           //If right has 
more space, turn right
    turnDir = 2;
  return turnDir;
