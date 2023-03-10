// Librarys
#include "PinChangeInterrupt.h"
#include <Wire.h>
//Setup Pins
#define GAIN_A 7
#define GAIN_B 7
//Enscoder Pins
#define MotorAEncoder 10
#define MotorBEncoder 11
#define distTolerance 3
  int left=0;
  int right=0;
   int first=0;
//Encoder count setup
volatile unsigned int EncoderCountA = 0;
volatile unsigned int EncoderCountB = 0;

// Defining these allows us to use letters in place of binary when
// controlling our motors
// Left motor == A
#define A 0
// Right motor == B
#define B 1

#define motorAfor 2
#define motorArev 3
#define motorBfor 5
#define motorBrev 4
//Driver Pin varables

// Drive constants - dependent on robot configuration
#define EncoderCountsPerRev 2240.0
#define DistancePerRev      31.9
#define DegreesPerRev       28.25

#define button 50
// Min PWM for robot's wheels to move
#define deadband_A 100
#define deadband_B 100
#define interruptPin 18
void indexEncoderCountA()
{
  EncoderCountA++;
}
//////////////////////////////////////////////////////////
void indexEncoderCountB()
{
  EncoderCountB++;
}    
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
void receiveData(int byteCount) {
while (Wire.available()) {
    // read twice because it is sending an extra 0 between data sends
    
    left = Wire.read();
    
    right = Wire.read();}}
// int motor is the defined A or B
// pwm = the power cycle you want to use
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

//IR setup (for testing)    
void setup() {
  motor_setup();
  // setup serial printing
  Serial.begin(9600);
  #define SLAVE_ADDRESS 0x09
  // put your setup code here, to run once:
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);

  while(digitalRead(button) == 0);
  Serial.print("Start ");


  //Setup the PinChange Interrupts
  pinMode(MotorAEncoder, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(MotorAEncoder), indexEncoderCountA, CHANGE);

  pinMode(MotorBEncoder, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(MotorBEncoder), indexEncoderCountB, CHANGE);
  //Estop interupt
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), Estop, LOW);
}

void loop() {
  Serial.print(left);
  Serial.print(right);
  if(left== 1 && right ==1 ){  
    Serial.print("stopping");

    run_motor(A, -255);
    run_motor(B, -255);
    delay(100);
    // we will use this as a way to stop the robot and end a test run
    run_motor(A, 0);
    run_motor(B, 0);
    
    //turn 180
    while(digitalRead(button) == 0);
  }
  
  else if(left == 1){
    Serial.print(" avoiding obstacle to left ");
    //turn right
    run_motor(A, 255);
    run_motor(B, -255);
    delay(100);
    run_motor(A, 0);
    run_motor(B, 0);
    delay(500);
  }
  else if(right == 1){
    Serial.print("avoiding obstacle to right");
    //turn left
    run_motor(A, -255);
    run_motor(B, 255);
    delay(100);
    run_motor(A, 0);
    run_motor(B, 0);
    delay(500);

    // go straight for a bit

    // turn right
  }
  
  else{
    Serial.print("straight");
    // go straight
    if (first==0){
    drive(10);
    first=1;
    run_motor(A, 0);
    run_motor(B, 0);
    }
    else{  
    Serial.print("stopping");
    // we will use this as a way to stop the robot and end a test run
    run_motor(A, 0);
    run_motor(B, 0);
    first=0;
    //turn 180
    while(digitalRead(button) == 0);
  }
    //run_motor(A, 255);
    //run_motor(B, 255);
    //Serial.print(EncoderCountA);
    //delay(1000);
    //run_motor(A, 0);
    //run_motor(B, 0);
    //Serial.print(EncoderCountA);
    //delay(1000);
  }
  
}
int drive(float distance)
{
  // create variables needed for this function
  int countsDesired, cmdLeft, cmdRight, errorLeft, errorRight;

  // Find the number of encoder counts based on the distance given, and the 
  // configuration of your encoders and wheels
  countsDesired = distance*(EncoderCountsPerRev/DistancePerRev)/70;

  // reset the current encoder counts
  EncoderCountA = 0;
  EncoderCountB = 0;
  
  // we make the errors greater than our tolerance so our first test gets us into the loop
  errorLeft = distTolerance + 1;
  errorRight =  distTolerance + 1;

  // Begin PID control until move is complete
  while (errorLeft > distTolerance || errorRight > distTolerance)
  {
    // Get PWM values from proportionalControl function
    cmdLeft = proportionalControl(GAIN_A, deadband_A, errorLeft);
    cmdRight = proportionalControl(GAIN_B, deadband_B, errorRight);

    // Set new PWMs
    run_motor(A, cmdLeft);
    run_motor(B, cmdRight);

    // Update encoder error
    // Error is the number of encoder counts between here and the destination
    errorLeft = countsDesired - EncoderCountA;
    errorRight = countsDesired - EncoderCountB;

    // If using bump sensors, check here for collisions
    // and call correction function

    /* Some print statements, for debugging*/
//    Serial.print(errorLeft);
//    Serial.print(" ");
//    Serial.print(cmdLeft);
//    Serial.print("\t");
//    Serial.print(errorRight);
//    Serial.print(" ");
//    Serial.println(cmdRight);

  }
}
////////////////////////////////////////////////////////////////////////////////


// Write a function for turning with PID control, similar to the drive function


//////////////////////////////////////////////////////////


// If you want, write a function to correct your path due 
// to an active bump sensor. You will want to compensate 
// for any wheel encoder counts that happend during this maneuver


//////////////////////////////////////////////////////////
int proportionalControl(int gain, int deadband, int error)
//  gain, deadband, and error, both are integer values
{
  if (error <= distTolerance) { // if error is acceptable, PWM = 0
    return (0);
  }

  int pwm = (gain * error); // Proportional control
  pwm = constrain(pwm,deadband,255); // Bind value between motor's min and max
  return(pwm);
  // Consider updating to include differential control
}
void Estop() {
  run_motor(A, 0);
  run_motor(B, 0);
  Serial.print("Estop Activated ");
  
  
}
