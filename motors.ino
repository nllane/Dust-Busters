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
