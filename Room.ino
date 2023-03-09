// Librarys
#include "PinChangeInterrupt.h"
#include <Wire.h>
//Setup Pins
#define KPA 10
#define KPB 10
#define KIA 0.1
#define KIB 0.1
//Enscoder Pins
#define MotorAEncoder 10
#define MotorBEncoder 11
#define distTolerance 3
  int room[]={0};
  int left=0;
  int right=0;
   int first=0;
//Encoder count setup
volatile unsigned int EncoderCountA = 0;
volatile unsigned int EncoderCountB = 0;

// Defining these allows us to use letters in place of binary when
// controlling our motors
// Right motor == A
#define A 0
// Left motor == B
#define B 1

#define motorAfor 2
#define motorArev 3
#define motorBfor 5
#define motorBrev 4
//Driver Pin varables

// Drive constants - dependent on robot configuration
#define EncoderCountsPerRev 1393.0
#define DistancePerRev      31.9
#define DegreesPerRev       100

#define button 50
// Min PWM for robot's wheels to move
#define deadband_A 67
#define deadband_B 67
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
  
  while(digitalRead(button) == 0);
  Serial.print("Start ");


  //Setup the PinChange Interrupts
  pinMode(MotorAEncoder, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(MotorAEncoder), indexEncoderCountA, CHANGE);

  pinMode(MotorBEncoder, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(MotorBEncoder), indexEncoderCountB, CHANGE);
  
}

void loop() {
  for (int i=0 ;i <sizeof(room);i++ ){
    if (room[i]==0){
      Serial.print("turn ");
      turn(0);
  }
  if (room[i]==1){
    turn(1);
  }
  else{
    Serial.print("drive ");
    drive (room[i]);
    Serial.print("Stop ");
    run_motor(B, 0);
    run_motor(A, 0);
    Serial.print(EncoderCountA);
    Serial.print(" ");
    Serial.print(EncoderCountB);
    while(digitalRead(button) == 0);{
  }
  }
  while(digitalRead(button) == 0);{
  }}
}
void turn(int way){
  // create variables needed for this function
  int countsDesired, cmdA, cmdB, errorA, errorB,totalErrorB,totalErrorA;

  // Find the number of encoder counts based on the distance given, and the 
  // configuration of your encoders and wheels
  float distance = (90*(1/DistancePerRev)*DistancePerRev);
  
  countsDesired = distance*(EncoderCountsPerRev/DistancePerRev);

  // reset the current encoder counts
  EncoderCountA = 0;
  EncoderCountB = 0;
  
  // we make the errors greater than our tolerance so our first test gets us into the loop
  errorA = distTolerance + 1;
  errorB =  distTolerance + 1;
  totalErrorA=errorA;
  totalErrorB=errorB;

  // Begin PID control until move is complete
  while (errorA > distTolerance || errorB > distTolerance)
  {
    // Get PWM values from proportionalControl function
    cmdA = PIControl(KPA,KIA, deadband_A,totalErrorA, errorA,255);
    cmdB = PIControl(KPB,KIB, deadband_B,totalErrorB, errorB,255);

    // Set new PWMs
    if (way == 1) {
      run_motor(A, cmdA);
      run_motor(B, -cmdB);
    } else if (way == 0) {
      run_motor(A, -cmdA);
      Serial.print(cmdA);
      Serial.print(" ");
      Serial.print(errorA);
      Serial.print(" A");
      Serial.print("\n");
      run_motor(B, cmdB);
      Serial.print(cmdB);
      Serial.print(" ");
      Serial.print(errorB);
      Serial.print(" B");
      Serial.print("\n");
    }
    
    // Update encoder error
    // Error is the number of encoder counts between here and the destination
    errorA = countsDesired - EncoderCountA;
    errorB = countsDesired - EncoderCountB;

//    Serial.print(errorA);
//    Serial.print(" ");
//    Serial.print(cmdA);
//    Serial.print("\t");
//    Serial.print(errorB);
//    Serial.print(" ");
//    Serial.println(cmdB);

  }
  
    delay(100);
    run_motor(A, 0);
    run_motor(B, 0);
}


int drive(float distance)
{
  // create variables needed for this function
  int countsDesired, cmdA, cmdB, errorA, errorB,totalErrorB,totalErrorA;

  // Find the number of encoder counts based on the distance given, and the 
  // configuration of your encoders and wheels
  countsDesired = distance*(EncoderCountsPerRev/DistancePerRev)*1.03;
Serial.print(countsDesired);
  // reset the current encoder counts
  EncoderCountA = 0;
  EncoderCountB = 0;
  
  // we make the errors greater than our tolerance so our first test gets us into the loop
  errorA = distTolerance + 1;
  errorB =  distTolerance + 1;

  totalErrorB = errorB;
  totalErrorA = errorA;

  // Begin PID control until move is complete
  while (errorA > distTolerance || errorB > distTolerance)
  {
    // Get PWM values from proportionalControl function
    cmdA = PIControl(KPA,KIA, deadband_A,totalErrorA, errorA,240);
    cmdB = PIControl(KPB,KIB, deadband_B,totalErrorB, errorB,255);

    // Set new PWMs
    run_motor(B, cmdB);
    delay(25);
    run_motor(A, cmdA);

    // Update encoder error
    // Error is the number of encoder counts between here and the destination
    errorA = countsDesired - EncoderCountA-110;
    errorB = countsDesired - EncoderCountB;

    totalErrorB += errorB;
    totalErrorA += errorA;
    // If using bump sensors, check here for collisions
    // and call correction function

    /* Some print statements, for debugging*/
//    Serial.print(errorA);
//    Serial.print(" ");
//    Serial.print(cmdA);
//    Serial.print("\t");
//    Serial.print(errorB);
 //   Serial.print(" ");
//    Serial.println(cmdB);

  }
  //Serial.print(errorA);
   // Serial.print(" ");
   // Serial.print(errorB);
}
////////////////////////////////////////////////////////////////////////////////


// Write a function for turning with PID control, similar to the drive function


//////////////////////////////////////////////////////////


// If you want, write a function to correct your path due 
// to an active bump sensor. You will want to compensate 
// for any wheel encoder counts that happend during this maneuver


//////////////////////////////////////////////////////////
int PIControl(int KP,float KI, int deadband,int totalError, int error, int max_speed)
//  gain, deadband, and error, both are integer values
{
  if (error <= distTolerance) { // if error is acceptable, PWM = 0
    return (0);
  }

  int pwm = (KP * error+KI*totalError); // Proportional control
  pwm = constrain(pwm,deadband,max_speed); // Bind value between motor's min and max
  return(pwm);
  // Consider updating to include differential control
}
