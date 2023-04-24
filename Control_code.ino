// Librarys
#include "PinChangeInterrupt.h"
#include <Wire.h>
//Setup Pins
#define GAIN_A 7
#define GAIN_B 7
//LED
#define Blue 27
#define Green 25
#define Red 29
#define Yellow 31

//Encoder Pins
#define MotorAEncoder 10
#define MotorBEncoder 11
//inicilaze values
#define distTolerance 3
  int left=0;
  int right=0;
  int dist=0;
  int angle=0;
  int lastdist=0;
  int high=0;
  int count=0;
  int low=0;
  int distdesired=50;
  int correction=0;
  int test=0;
//Encoder count setup
volatile unsigned int EncoderCountA = 0;
volatile unsigned int EncoderCountB = 0;

// Defining these allows us to use letters in place of binary when
// controlling our motors
// Left motor == A
#define A 0
// Right motor == B
#define B 1
//left
#define motorAfor 2
#define motorArev 3
//right pins
#define motorBfor 5
#define motorBrev 4
//Driver Pin varables
#define vacuum 28
#define brush 7
// Drive constants - dependent on robot configuration
#define EncoderCountsPerRev 2240.0
#define DistancePerRev      31.9
#define DegreesPerRev       28.25

#define button 23
// Min PWM for robot's wheels to move
#define deadband_A 100
#define deadband_B 100
#define interruptPin 18
#define errorPin 19
//////////////////////////////////////////////////////////
void indexEncoderCountA()
{//encoders for motor A
  EncoderCountA++;
}
//////////////////////////////////////////////////////////
void indexEncoderCountB()
{//encoders for motor B
  EncoderCountB++;
}  
//////////////////////////////////////////////////////////
void run_motor(int motor, int pwm) {
  int dir = (pwm/abs(pwm))> 0; // returns if direction is forward (1) or reverse (0)
  pwm = abs(pwm); // only positive values can be sent to the motor
    switch (motor) { // find which motor to control
      case A: // if A, write A pins
        if (dir == 1){
          digitalWrite(motorAfor, pwm); // dir is either 1 (forward) or 0 (reverse)
          analogWrite(motorArev, 0); // pwm is an analog value 0-255
          }
        else {
          digitalWrite(motorAfor, 0); // dir is either 1 (forward) or 0 (reverse)
          analogWrite(motorArev, pwm); // pwm is an analog value 0-255
          }
        break; // end case A
      case B: // if B, write B pins
        if (dir == 1){
          digitalWrite(motorBfor, pwm); // dir is either 1 (forward) or 0 (reverse)
          analogWrite(motorBrev, 0); // pwm is an analog value 0-255
          }
        else {
          digitalWrite(motorBfor, 0); // dir is either 1 (forward) or 0 (reverse)
          analogWrite(motorBrev, pwm); // pwm is an analog value 0-255
          }
        break; // end case B
    } // end switch statement

  return;
} // end function
//////////////////////////////////////////////////////////
//IR setup (for testing)      
void motor_setup() {
    //define Sheild pins as outputs
    pinMode(motorAfor, OUTPUT); 
    pinMode(motorArev, OUTPUT);
    pinMode(motorBfor, OUTPUT);
    pinMode(motorBrev, OUTPUT);
    // initialize all pins to zero
    digitalWrite(motorAfor, 0);
    digitalWrite(motorArev, 0);
    digitalWrite(motorBfor, 0);
    digitalWrite(motorBrev, 0);
  return;
} // end function
//////////////////////////////////////////////////////////
//stop the robot
void Estop() {
  delay(25);
  run_motor(A, 0);//set motors to not run
  run_motor(B, 0);
  test=1;
  digitalWrite(vacuum, LOW);// turn off vacuum
  digitalWrite(brush, LOW);
  Serial.print("Estop Activated ");
  while(digitalRead(button) == HIGH); //delay till reset
  delay(1000);
  Serial.print("restarted ");
  digitalWrite(brush, HIGH);
  digitalWrite(vacuum, HIGH); //turn vacuum on
}

void error() {
  delay(25);
  run_motor(A, 0);//set motors to not run
  run_motor(B, 0);
  digitalWrite(Green, HIGH);
  digitalWrite(vacuum, LOW);// turn off vacuum
  digitalWrite(brush, LOW);
  Serial.print("error");
  //while(digitalRead(errorPin) == HIGH);
  while(digitalRead(button) == HIGH); //delay till reset
  digitalWrite(Green, LOW);
}

//////////////////////////////////////////////////////////
void receiveData(int byteCount) {
  //recive position from pi
while (Wire.available()) {
    // read five values for positioning
    correction = Wire.read();
    right = Wire.read();
    left = Wire.read();
    dist = Wire.read()+255*correction;//adjusts istance to fit bit size
    angle=Wire.read()+255; //adjusts angle to fit bit size
    }}
//////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
void setup() {
  pinMode(button, INPUT_PULLUP);
  // setup collection pins
  pinMode(vacuum, OUTPUT);
  pinMode(brush, OUTPUT);
  //LED pin setup
  pinMode(Blue, OUTPUT); 
  pinMode(Green, OUTPUT);
  pinMode(Red, OUTPUT);
  pinMode(Yellow, OUTPUT);
    // initialize all pins to zero
  digitalWrite(Blue, 0);
  digitalWrite(Green, 0);
  digitalWrite(Red, 0);
  digitalWrite(Yellow, 0);
  motor_setup(); // run motor setup
  // setup serial printing
  Serial.begin(9600);
  // put your setup code here, to run once:
  //set up the arduino to use i2c communication
  #define SLAVE_ADDRESS 0x09
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);

  //delay till start activated
  while(digitalRead(button) == HIGH);
  delay(1000);
  Serial.print("Start ");
  digitalWrite(vacuum, HIGH);
  digitalWrite(brush, HIGH);//set high after start button
   //Setup the PinChange Interrupts
  pinMode(MotorAEncoder, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(MotorAEncoder), indexEncoderCountA, CHANGE);

  pinMode(MotorBEncoder, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(MotorBEncoder), indexEncoderCountB, CHANGE);
  //Estop interupt
// pinMode(errorPin, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(errorPin), error, LOW);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), Estop, LOW);


}

void loop() {
  //uses the values provided by lidar to adjust movment
  //both left side or right blocked and to close to a wall
  if(left== 1 ||(right==1&&dist<100) ){  
    Serial.print("large turn \n");

    run_motor(A, 100); //turn right
    run_motor(B, -100);
    delay(100);}
    
    //turn 
//far off the wall
  else if(dist>(distdesired*1.5)){
    Serial.print("far away");
    
    while (angle <280){ 
    run_motor(A, -100);//turn left
    run_motor(B, 100);//turn left
    delay(20);}
    run_motor(A, 255);//go forward
    run_motor(B, 255);
delay(80);
    //turn left
  } 
  else if(angle>280){
    Serial.print("too angled \n");
    
    run_motor(A, 100);//turn right
    run_motor(B, -100);
    delay(50);
    run_motor(A, 0); //wait to reduce over shoot
    run_motor(B, 0);
    delay(50);
    }
  else if((distdesired<=dist&&dist<=(distdesired+5))||((distdesired+5)&&(angle<280&&angle>275))){
    Serial.print("straight \n");
    while (angle >273){ 
      if (dist>70){//avoid infinite loop
        break;}
    run_motor(A, 100); //turn right until parellel
    run_motor(B, -100);
    delay(10);}
    // go straight
    run_motor(A, 255);
    run_motor(B, 255);
    delay(50);
    }
//close to wall
  else if(distdesired>dist){  
    Serial.print("Correct right\n");
    run_motor(A, 255);
    run_motor(B, -10); //none pinpoint turn
    delay(50);
    //small turn right
    run_motor(A, 255);
    run_motor(B, 255);
    delay(50);
    //go forward
  }
//away from wall
  else if(dist>(distdesired+5)){  
    Serial.print("Correct left\n");
    run_motor(A, -10);
    run_motor(B, 255);//none pinpoint turn
    //turn left
    delay(50);
    //small turn right
    run_motor(A, 255);
    run_motor(B, 255);
    delay(50);
    }
    //go forward
    
//inconsistant recive something is wrong
  else{
    //Serial.print("stopping\n");
    Serial.print("broke");
    run_motor(A, 0);
    run_motor(B, 0);
    delay(200);
    while(digitalRead(button) == HIGH);
  delay(1000);}
    count++;
    if (count==10){
      checkSensers();
      count=0;
      }

    Serial.print("\n");
    Serial.print("\n");
    Serial.print("next");
    Serial.print("\n");
  }
  void checkSensers(){
float AvgAcs=0,AcsValue=0.0,Samples=0.0,vacCurrent=0.0,brushCurrent=0.0,Volt=0.0,vacuumVolt=0.0,RCurrent=0.0,LCurrent=0.0,AcsValueF=0.0;
Serial.print("Check");
delay(2000);

// brush A5
run_motor(A, 100);
run_motor(B, 100);
  for (int x = 0; x < 10; x++){ //Get 150 samples
  AcsValue = analogRead(A5);     //Read current sensor values   
  Samples = Samples + AcsValue;  //Add samples together
  delay (3); // let ADC settle before next sample 3ms
}
AvgAcs=Samples/10.0;//Taking Average of Samples
brushCurrent = (2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.070;
Samples=0.0;
//vac current A3
for (int x = 0; x < 10; x++){ 
  AcsValue = analogRead(A3);     
  Samples = Samples + AcsValue; 
  delay (3); 
}
AvgAcs=Samples/10.0;//Taking Average of Samples
vacCurrent = (2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.070;
Samples=0.0;
//L current A2
for (int x = 0; x < 10; x++){ 
  AcsValue = analogRead(A2);   
  Samples = Samples + AcsValue;  
  delay (3); 
}
AvgAcs=Samples/10.0;
LCurrent= (2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.070;
Samples=0.0;

//R current A1
for (int x = 0; x < 10; x++){ //Get 150 samples
  AcsValue = analogRead(A1);     //Read current sensor values   
  Samples = Samples + AcsValue;  //Add samples together
  delay (3); // let ADC settle before next sample 3ms
}
AvgAcs=Samples/10.0;//Taking Average of Samples
RCurrent= (2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.070;
Samples=0.0;
// Original 0.066
vacuumVolt = analogRead(A4)*(5/1023)*5.1;
Volt= analogRead(A6)*(5/1023)*5.1;
if (brushCurrent >2.9){
  Serial.print("brush Stall");
    digitalWrite(Blue, HIGH);
    stop();
    digitalWrite(Blue, LOW);}
    
if (RCurrent>6||LCurrent>6){
  Serial.print("Movment Stall");
    digitalWrite(Red, HIGH);
    stop();
    digitalWrite(Red, LOW);}

if (vacCurrent>24){
  Serial.print("Vacuum Stall");
    digitalWrite(Green, HIGH);
    stop();
    digitalWrite(Green, LOW);}
if (vacuumVolt<16.2||Volt<16.2){
  digitalWrite(Yellow, HIGH);
    stop();
    digitalWrite(Yellow, LOW);}

   Serial.println(vacCurrent);
   Serial.print("\n");
    Serial.print("\n");
   Serial.println(RCurrent);
   Serial.print("\n");
    Serial.print("\n");
   Serial.println(LCurrent);
   Serial.print("\n");
    Serial.print("\n");
   Serial.println(brushCurrent); 
   Serial.print("\n");
    Serial.print("\n");
   Serial.println(brushCurrent); 
  digitalWrite(brush, HIGH);
  digitalWrite(vacuum, HIGH);
}
void stop(){
  
    digitalWrite(brush, LOW);
    digitalWrite(vacuum, LOW);
    run_motor(A, 0);
    run_motor(B, 0);
    while(digitalRead(button) == HIGH);
    digitalWrite(brush, HIGH);
    digitalWrite(vacuum, HIGH);}
