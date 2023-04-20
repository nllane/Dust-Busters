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
//initialize values
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

// Defining these allows us to use letters in place of binary when controlling the motors
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
//Collection variables
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
    //define motor pins as outputs
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
  Serial.print("restarted ");
  digitalWrite(brush, HIGH);
  digitalWrite(vacuum, HIGH); //turn vacuum on
}

//////////////////////////////////////////////////////////
void receiveData(int byteCount) {
  //Receive position from pi
while (Wire.available()) {
    // read five values for positioning
    correction = Wire.read();
    right = Wire.read();
    left = Wire.read();
    dist = Wire.read()+255*correction;//readjusts because of byte size bit size
    angle=Wire.read()+255; //adjusts angle to fit bit size
    }}
//////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
void setup() {
  pinMode(Blue, OUTPUT); 
  pinMode(Green, OUTPUT);
  pinMode(Red, OUTPUT);
  pinMode(Yellow, OUTPUT);
    // initialize all pins to zero
  digitalWrite(Blue, 0);
  digitalWrite(Green, 0);
  digitalWrite(Red, 0);
  digitalWrite(Yellow, 0);
  pinMode(button, INPUT_PULLUP);
  // setup collection pins
  pinMode(vacuum, OUTPUT);
  pinMode(brush, OUTPUT);
  motor_setup();
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
  Serial.print("Start ");
  digitalWrite(vacuum, HIGH);
  digitalWrite(brush, HIGH);//stet high when new motors


  //Setup the PinChange Interrupts
  pinMode(MotorAEncoder, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(MotorAEncoder), indexEncoderCountA, CHANGE);

  pinMode(MotorBEncoder, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(MotorBEncoder), indexEncoderCountB, CHANGE);
  //Estop interrupt
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), Estop, LOW);
}

void loop() {
  //uses the values provided by lidar to adjust movement
  //both left side or right blocked and to close to a wall
  if(left== 1 ||(right==1&&dist<100) ){  
    Serial.print("large turn \n");

    run_motor(A, 100); //turn right
    run_motor(B, -100);
    delay(20);}
    
    //turn 
//large distance off the wall
  else if(dist>(distdesired*2)){
    Serial.print("return to wall\n");
    
    while (angle <280){ 
    run_motor(A, -100);//turn left
    run_motor(B, 100);//turn left
    delay(20);}
    run_motor(A, 255);//go forward
    run_motor(B, 255);
delay(100);
  } 
  else if(angle>280){
    Serial.print("too angled \n");
    
    run_motor(A, 100);//turn right
    run_motor(B, -100);
    delay(10);
    run_motor(A, 0); //wait to reduce over shoot
    run_motor(B, 0);
    delay(10);
    }
  else if((distdesired<=dist&&dist<=(distdesired+5))||((dist<=distdesired+5)&&(angle<280&&angle>275))){
    Serial.print("straight \n");
    while (angle >273){ 
      if (dist>70){//to avoid infinite loop
        break;}
    run_motor(A, 100); //turn right
    run_motor(B, -100);
    delay(10);}
    // go straight
    run_motor(A, 255);
    run_motor(B, 255);
    delay(50);
    }
//to close to wall
  else if(distdesired>dist){  
    Serial.print("Correct right\n");
    run_motor(A, 255);
    run_motor(B, -10); //none pinpoint turn
    delay(50);
    // go forward to make a less drastic turn
    run_motor(A, 255);
    run_motor(B, 255);
    delay(50);
    //go forward
  }
//small distance away from wall
  else if(dist>(distdesired+5)){  
    Serial.print("Correct left\n");
    run_motor(A, -10);
    run_motor(B, 255);//none pinpoint turn
    //turn left
    delay(50);
    // go forward 
    run_motor(A, 255);
    run_motor(B, 255);
    delay(50);
    }
    //go forward
    
//An inconsistent receive indicates something is wrong
  else{
    //Serial.print("stopping\n");
    Serial.print("broken");
    run_motor(A, 0);//stops motors
    run_motor(B, 0);
    //delays till reset
    while(digitalRead(button) == HIGH);
    delay(200);}
// prints to debug
//    Serial.print(angle);
//    Serial.print("\n");
//    lastdist=dist;
//    Serial.print(dist);
//    Serial.print("\n");
//    Serial.print("\n");
//    Serial.print("next");
//    Serial.print("\n");
  }
