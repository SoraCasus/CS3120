  #include <AFMotor.h>

#define _LEFT_ false
#define _RIGHT_ true

  AF_DCMotor motorR(1);
  AF_DCMotor motorL(2);

  const uint16_t LEFT_SPEED = 170;
  const uint16_t RIGHT_SPEED = 200;

void setup() {
  // put your setup code here, to run once:
  
  motorL.setSpeed(LEFT_SPEED);
  motorR.setSpeed(RIGHT_SPEED);

  motorL.run(RELEASE);
  motorR.run(RELEASE);

}

void Turn(bool right) {
  if(right) {
    motorL.run(FORWARD);
    motorR.run(BACKWARD);

    delay(215);

    motorL.run(RELEASE);
    //delay(5);
    motorR.run(RELEASE);
  } else {
    motorL.run(BACKWARD);
    motorR.run(FORWARD);

    delay(250);

    motorL.run(RELEASE);
    //delay(5);
    motorR.run(RELEASE);
  }
}

void MoveForward() {

  motorL.setSpeed(225);
  motorR.setSpeed(180);
  
  motorR.run(FORWARD);
  delay(50);
  motorL.run(FORWARD);

  delay(1200);
  
  motorR.run(RELEASE);
  // delay(5);
  motorL.run(RELEASE);
  
  motorL.setSpeed(LEFT_SPEED);
  motorR.setSpeed(RIGHT_SPEED);
}

void loop() {
  // put your main code here, to run repeatedly:
  MoveForward();
  delay(500);
  Turn(_RIGHT_);
  delay(500);
  MoveForward();
  delay(500);
  Turn(_RIGHT_);
  delay(500);
  MoveForward();
  delay(500);
  Turn(_RIGHT_);
  delay(500);
  MoveForward();
  delay(500);
  Turn(_RIGHT_);
  
  while(true);

}
