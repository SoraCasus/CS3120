  #include <AFMotor.h>

  AF_DCMotor motorL(1);
  AF_DCMotor motorR(2);

  const uint16_t left_speed = 170;
  const uint16_t right_speed = 200;

void setup() {
  // put your setup code here, to run once:
  
  motorL.setSpeed(left_speed);
  motorR.setSpeed(right_speed);

  motorL.run(RELEASE);
  motorR.run(RELEASE);

}

void loop() {
  // put your main code here, to run repeatedly:

  motorL.run(FORWARD);
  motorR.run(BACKWARD);

  delay(280);

  motorL.run(RELEASE);
  delay(5);
  motorR.run(RELEASE);

  delay(2000);

  motorL.run(BACKWARD);
  motorR.run(FORWARD);

  delay(280);

  motorL.run(RELEASE);
  delay(5);
  motorR.run(RELEASE);

  while(true);

}
