
#define button 50
  #include <Wire.h>
   int in=0;
   int input=0;
  int first=1;
  int left=0;
  int right=0;
void setup() {
  pinMode(2, OUTPUT);
  //left motor
  pinMode(3, OUTPUT);
  //right motor
  // put your setup code here, to run once:
  #define TIME 2000
  #define SLAVE_ADDRESS 0x09
  Serial.begin(19200);
  // put your setup code here, to run once:
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);

  while(digitalRead(button) == 0);
  Serial.print("Start ");
}

void loop() {
  //Ackerman recommended using an accelorometer/gyroscope module in order to keep track of location
  //which would allow us to create a "map" inside the RPi and avoid using tape on the floor
  //the LiDAR would scan the room and "update" the map based on what it sees, and it would know where
  //the robot is at based on the gyro readings and where the "home base" is at
  
  // "in" is the bit read from RPi. LiDAR logic:
  // 0 is no obstacle detected (or it's far away)
  // 1 means there is an obstacle to left (between 0 and 50 degrees)
  // 2 means there is an obstacle to right (between 310 and 360 degrees)
  
  if(left== 1 && right ==1 ){  
    Serial.print("stopping");
    // we will use this as a way to stop the robot and end a test run
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    
    //turn 180
    while(digitalRead(button) == 0);
  }
  else if(left == 1){
    Serial.print(" avoiding obstacle to left ");
    //turn right
    //for(int i = 0; i < 15; i++){
    //digitalWrite(2, HIGH);
    //digitalWrite(3, LOW);
    //delay(100);
    //digitalWrite(2, LOW);
    //digitalWrite(3, LOW);
    //}
    
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    delay(1500);
    //digitalWrite(2, LOW);
    //digitalWrite(3, HIGH);
    //delay(1000);

    // go straight for a bit

    // turn left
    left = 0;
  }
  else if(right == 1){
    Serial.print("avoiding obstacle to right");
    //turn left
    //digitalWrite(2, LOW);
    //digitalWrite(3, HIGH);
    //delay(1000);
    //digitalWrite(2, HIGH);
    //digitalWrite(3, HIGH);
    //delay(1000);
    //digitalWrite(2, HIGH);
    //digitalWrite(3, LOW);
    //delay(1000);

    // go straight for a bit

    // turn right
    right = 0;
  }
  
  else{
    Serial.print("straight");
    // go straight
    //digitalWrite(2, HIGH);
    //digitalWrite(3, HIGH);
    
    delay(1000);
  }
digitalWrite(3, LOW);
digitalWrite(2, HIGH);
    delay(3000);
digitalWrite(2, LOW);
digitalWrite(3, LOW);
    delay(1000);
digitalWrite(2, HIGH);
digitalWrite(3, HIGH);
    delay(1000);
digitalWrite(2, LOW);
digitalWrite(3, LOW);
    delay(1000);
digitalWrite(3, LOW);
digitalWrite(2, HIGH);
    delay(3000);
digitalWrite(2, LOW);
digitalWrite(3, LOW);
    delay(1500);
delay(400);
 while(digitalRead(button) == 0);
}
void receiveData(int byteCount) {
while (Wire.available()) {
    // read twice because it is sending an extra 0 between data sends
    
    left = Wire.read();
    
    right = Wire.read();    
}}
