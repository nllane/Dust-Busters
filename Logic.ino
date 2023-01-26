
void setup() {
  // put your setup code here, to run once:
  #define TIME 2000
  #include <Wire.h>
  int in=0;
  int first=1;
  #define SLAVE_ADDRESS 0x09
  Serial.begin(19200);
  // put your setup code here, to run once:
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
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
  int in =0;
  switch(in){
    case(0):
      //drive forward
      //set constant motor speed
      Serial.print("Forward");
      delay(500);
    case(1):
      //avoid obstacle to left
      Serial.print("Avoiding Obstacle to Left");
      
      // "Turn"
      //drive left motor faster
      //drive right motor slower
      delay(TIME);
      // "Straighten out"
      //drive left motor slower
      //drive right motor faster
      delay(TIME);
      in = 0;
      
     case(2):
     //avoid obstacle to right
     Serial.print("Avoiding Obstacle to Right");

     // "Turn"
     //drive left motor slower
     //drive right motor faster
     delay(TIME);
     // "Straighten out"
     //drive left motor faster
     //drive right motor slower
     delay(TIME);
     in = 0;
  }

}
void receiveData(int byteCount) {
while (Wire.available()) {
    // read twice because it is sending an extra 0 between data sends
    in = Wire.read();
    in = Wire.read();
}}
