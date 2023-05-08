/*
This code is to be run with the Vision_Main.py 
This is the code run at the showcase and loaded on the Arduino upon delivery.
This code uses only the distance provided by the lidar to fallow the walls.
The Estop and error interrupts are not reliable but workable.
I believe that the cause of the reliability issue is the I2C communication interfering with the pin but did not figure out a fix.
Sensing needs to have the values tested for fine tuning
Sensing was turned off for the show case because it requires the motor to be running which meant that it goes forward for around .12 seconds without vision. I believe this can be optimized more efficiently
This code arks its wall following and struggles with hollow objects.
*/


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
//change distdesired to change position off of wall
  int distdesired=50;
  int left=0;
  int right=0;
  int dist=0;
  int angle=0;
  int lastdist=0;
  int high=0;
  int count=0;
  int low=0;
  int correction=0;
  //test was to stop code after a set number of runs
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
 //The Estop was being interfered with by the I2C communication.
  //when run without the I2C sending It worked consistently.
  
  delay(25);
  run_motor(A, 0);//set motors to not run
  run_motor(B, 0);
  test=1;
  digitalWrite(vacuum, LOW);// turn off vacuum
  digitalWrite(brush, LOW);
    digitalWrite(Blue, HIGH);//set indicators on
  digitalWrite(Green, HIGH);
  digitalWrite(Red, HIGH);
  digitalWrite(Yellow, HIGH);
  Serial.print("Estop Activated ");
  while(digitalRead(button) == HIGH); //delay till reset
  delay(1000);
  Serial.print("restarted ");
    digitalWrite(Blue, LOW);
  digitalWrite(Green, LOW);
  digitalWrite(Red, LOW);   //turn off indicators 
  digitalWrite(Yellow, LOW);
  digitalWrite(brush, HIGH);
  digitalWrite(vacuum, HIGH); //turn vacuum on
}

//////////////////////////////////////////////////////////
void error() {
  //the raspberry pi sends 3.3V and the arduino is a 5V system so this was un reliable
  //it could work when the motors were disconected
  delay(25);
  run_motor(A, 0);//set motors to not run
  run_motor(B, 0);
  digitalWrite(Green, HIGH);// output the alert
  digitalWrite(vacuum, LOW);// turn off vacuum
  digitalWrite(brush, LOW);
  Serial.print("error");
  //while(digitalRead(errorPin) == HIGH);
  while(digitalRead(button) == HIGH); //delay till reset
  digitalWrite(Green, LOW);
  digitalWrite(brush, HIGH);
  digitalWrite(vacuum, HIGH); //turn vacuum on
}

//////////////////////////////////////////////////////////

void receiveData(int byteCount) {
  //Receive position from pi
while (Wire.available()) {
    // read five values for positioning
    correction = Wire.read();// 255 is the highest and the distance can be larger than tha
    right = Wire.read();
    left = Wire.read();
    dist = Wire.read()+255*correction;//readjusts because of byte size bit size
    angle=Wire.read()+255; //adjusts angle to fit bit size
    }}

//////////////////////////////////////////////////////////
void setup() {
pinMode(button, INPUT_PULLUP);
  // setup collection pins
  pinMode(vacuum, OUTPUT);
  pinMode(brush, OUTPUT);
  pinMode(Blue, OUTPUT); 
  pinMode(Green, OUTPUT);
  pinMode(Red, OUTPUT);
  pinMode(Yellow, OUTPUT);
    // initialize all pins to zero
  digitalWrite(Blue, LOW);
  digitalWrite(Green, LOW);
  digitalWrite(Red, LOW);
  digitalWrite(Yellow, LOW);
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
  delay(1000);
  Serial.print("Start ");
  digitalWrite(vacuum, HIGH);
  digitalWrite(brush, HIGH);//stet high when new motors
  
  //Setup the PinChange Interrupts
  pinMode(MotorAEncoder, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(MotorAEncoder), indexEncoderCountA, CHANGE);

  pinMode(MotorBEncoder, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(MotorBEncoder), indexEncoderCountB, CHANGE);
  //Estop interrupt
  // these are commented out because they were unreliable
  // If the code stays in an interupt then the I2C communication fails and then the python code hits an error
  pinMode(errorPin, INPUT);
  //attachInterrupt(digitalPinToInterrupt(errorPin), error, LOW);
  pinMode(interruptPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(interruptPin), Estop, LOW);

  }

void loop() {
  //uses the values provided by lidar to adjust movement
  //left side or right blocked and to close to a wall
  if (left== 1 ||(right==1&&dist<100) ){  
    Serial.print("large turn \n");

    run_motor(A, 100); //turn right
    run_motor(B, -100);
    delay(10);}
    
    //turn 
//large distance off the wall
 
    
  else if((distdesired<=dist&&dist<=(distdesired+5))){
  // activates if in the set distance
    Serial.print("straight \n");

    // go straight
    run_motor(A, 255);
    run_motor(B, 255);
    delay(20);
    }
//to close to wall
  else if(distdesired>dist){  
    Serial.print("Correct right\n");
    run_motor(A, 255);
    run_motor(B, 0); //none pinpoint turn
    delay(20);
    // go forward to make a less drastic turn
    run_motor(A, 255);
    run_motor(B, 255);
    delay(50);
    //go forward
  }
  else if(dist>(distdesired+100)){ 
    // this triggers if it sees a extra long distance to the wall
    Serial.print("meter off\n");
   run_motor(A, -255);
    run_motor(B, 255); //pinpoint turn
    delay(50);
    run_motor(A, 255);
    run_motor(B, 255);
    delay(50);
    // go forward 

    }
  
//small distance away from wall
  else if(dist>(distdesired+5)){  
    Serial.print("Correct left\n");
   run_motor(A, 0);
    run_motor(B, 255); //none pinpoint turn
    delay(20);
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
    delay(1000);}
//check status every second    
   count++;
   if (count==10){
      //check sensors is deactivated because it required the drive motor to be on making the robot go forward
      //checkSensers();
      count=0;
      }
    
// prints to debug
    Serial.print(angle);
    Serial.print("\n");
//    lastdist=dist;
//    Serial.print(dist);
//    Serial.print("\n");
//    Serial.print("next");
//    Serial.print("\n");
  }
   void checkSensers(){
float AvgAcs=0,AcsValue=0.0,Samples=0.0,vacCurrent=0.0,brushCurrent=0.0,Volt=0.0,vacuumVolt=0.0,RCurrent=0.0,LCurrent=0.0,AcsValueF=0.0;
Serial.print("Check");
delay(2000);

// brush A5
run_motor(A, 100);
run_motor(B, 100);
  for (int x = 0; x < 10; x++){ //Get 10 samples
  AcsValue = analogRead(A5);     //Read current sensor values   
  Samples = Samples + AcsValue;  //Add samples together
  delay (3); // let ADC settle before next sample 3ms
}
  AvgAcs=Samples/10.0;//Taking Average of Samples
  brushCurrent = (2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.070; //calculation to adjust for bit size and the senors values
Samples=0.0;
     
//vac current A3
for (int x = 0; x < 10; x++){ 
  AcsValue = analogRead(A3);     
  Samples = Samples + AcsValue; 
  delay (3); 
}
  AvgAcs=Samples/10.0;//Taking Average of Samples
  vacCurrent = (2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.070;  //calculation to adjust for bit size and the senors values
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

for (int x = 0; x < 10; x++){ 
  AcsValue = analogRead(A2);   
  Samples = Samples + AcsValue;  
  delay (3); 
}
  AvgAcs=Samples/10.0;
  LCurrent= (2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.070; //calculation to adjust for bit size and the senors values
Samples=0.0;

//R current A1
for (int x = 0; x < 10; x++){ //Get 10 samples
  AcsValue = analogRead(A1);     //Read current sensor values   
  Samples = Samples + AcsValue;  //Add samples together
  delay (3); // let ADC settle before next sample 3ms
}
  AvgAcs=Samples/10.0;//Taking Average of Samples
  RCurrent= (2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.070; //calculation to adjust for bit size and the senors values
Samples=0.0;
// Original 0.066

vacuumVolt = analogRead(A4)*(5/1023)*5.1;// bit correction for voltage sensors
Volt= analogRead(A6)*(5/1023)*5.1;// bit correction for voltage sensors

//checks the values aquired to the stall currents
//the values should be tested and changed acordingly we have repaced thing sense this was added
if (brushCurrent >2.9){ //3 is the stall current
  Serial.print("brush Stall");
    digitalWrite(Blue, HIGH); //turn on light
    stop(); //wait in a funtion till the issue is resolved
    digitalWrite(Blue, LOW);}//turn off light
    
if (RCurrent>6||LCurrent>6){//stall current was calculated to 8
  Serial.print("Movment Stall");
    digitalWrite(Red, HIGH);//turn on light
    stop();
    digitalWrite(Red, LOW);}

if (vacCurrent>24){   //stall current was calculated to 24
  Serial.print("Vacuum Stall");
    digitalWrite(Green, HIGH);
    stop();
    digitalWrite(Green, LOW);}
if (vacuumVolt<16.2||Volt<16.2){ // batteries hit one bar of charge at this voltage
  digitalWrite(Yellow, HIGH);
    stop();
    digitalWrite(Yellow, LOW);}

//   Serial.println(vacCurrent);
//   Serial.print("\n");
//   Serial.print("\n");
//   Serial.println(RCurrent);
//   Serial.print("\n");
//   Serial.print("\n");
//   Serial.println(LCurrent);
//   Serial.print("\n");
//   Serial.print("\n");
//   Serial.println(brushCurrent); 
//   Serial.print("\n");
//   Serial.print("\n");
//   Serial.println(brushCurrent); 
}
void stop(){
    //this is called is a sensor is set high could be used with estop to make it easyer to reset
    digitalWrite(brush, LOW);
    digitalWrite(vacuum, LOW);// stops the vacuum and brush
    run_motor(A, 0);
    run_motor(B, 0); //don't move
    while(digitalRead(button) == HIGH); //start with a new button press
    digitalWrite(brush, HIGH);
    digitalWrite(vacuum, HIGH);}//restarts the systems
  
