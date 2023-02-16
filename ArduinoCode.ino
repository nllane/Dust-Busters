// Librarys
#include "PinChangeInterrupt.h"

//Setup Pins

//Enscoder Pins
#define MotorAEncoder 7
#define MotorBEncoder 8

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
#define motorBfor 4
#define motorBrev 5
//Driver Pin varables

// Drive constants - dependent on robot configuration
#define EncoderCountsPerRev 12.0
#define DistancePerRev      4.55
#define DegreesPerRev       28.25


// Min PWM for robot's wheels to move
#define deadband_A 100
#define deadband_B 100


//IR setup (for testing)    
void setup() {
  motor_setup();

  // setup serial printing
  Serial.begin(9600);

  //Setup the PinChange Interrupts
  pinMode(MotorAEncoder, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(MotorAEncoder), indexEncoderCountA, CHANGE);

  pinMode(MotorBEncoder, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(MotorBEncoder), indexEncoderCountB, CHANGE);
  
}

void loop() {
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
  }
  else if(left == 1){
    Serial.print(" avoiding obstacle to left ");
    //turn right
    run_motor(A, 255);
    run_motor(B, -255);
    left = 0;
  }
  else if(right == 1){
    Serial.print("avoiding obstacle to right");
    //turn left
    run_motor(A, -255);
    run_motor(B, 255);

    // go straight for a bit

    // turn right
    right = 0;
  }
  
  else{
    Serial.print("straight");
    // go straight
    run_motor(A, 255);
    run_motor(B, 255);
    
  }
delay(1500);
}
    


//////////////////////////////////////////////////////////

// These are the encoder interupt funcitons, they should NOT be edited

void indexEncoderCountA()
{
  EncoderCountA++;
}
//////////////////////////////////////////////////////////
void indexEncoderCountB()
{
  EncoderCountB++;
}
void receiveData(int byteCount) {
while (Wire.available()) {
    // read twice because it is sending an extra 0 between data sends
    
    left = Wire.read();
    
    right = Wire.read();    
}}
