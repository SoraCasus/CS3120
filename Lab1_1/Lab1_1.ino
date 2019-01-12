#include <AFMotor.h>

AF_DCMotor motorL(1);
AF_DCMotor motorR(2);

const uint16_t LEFT_SPEED = 171;
const uint16_t RIGHT_SPEED = 200;

void setup() {
  // put your setup code here, to run once:

  // Set the speed of the motor
  motorL.setSpeed(LEFT_SPEED);
  motorR.setSpeed(RIGHT_SPEED);

  // Make sure the motors are off
  motorL.run(RELEASE);
  motorR.run(RELEASE);

}

void loop() {
  // put your main code here, to run repeatedly:

  motorR.run(FORWARD);
  motorL.run(FORWARD);
  
  delay(2700);
  
  motorR.run(RELEASE);
  delay(5);
  motorL.run(RELEASE);

  delay(500);

  motorR.run(BACKWARD);
  motorL.run(BACKWARD);

  delay(2700);

  motorR.run(RELEASE);
  delay(5);
  motorL.run(RELEASE);
  
  while(true);
}
