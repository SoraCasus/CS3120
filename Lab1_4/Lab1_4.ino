#include <AFMotor.h>

const uint8_t LASER_PIN = 14;
// const uint8_t LED_PIN = 15;
const uint8_t OPTO_PIN = A0;

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);

const uint8_t LEFT_SPEED = 80;
const uint8_t RIGHT_SPEED = 60;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(LASER_PIN, OUTPUT);
  // pinMode(LED_PIN, OUTPUT);
  pinMode(OPTO_PIN, INPUT);

  digitalWrite(LASER_PIN, HIGH);

    motorR.setSpeed(200);
    motorL.setSpeed(200);
    motorR.run(FORWARD);
    motorL.run(FORWARD);

    delay(50);

}

void loop() {
  // put your main code here, to run repeatedly:

  int32_t optoValue = analogRead(OPTO_PIN);

  motorR.setSpeed(RIGHT_SPEED);
  motorL.setSpeed(LEFT_SPEED);


    motorR.setSpeed(RIGHT_SPEED);
    motorL.setSpeed(LEFT_SPEED);
    
    motorR.run(FORWARD);
    delay(50);
    motorL.run(FORWARD);
  
  if(optoValue > 1018) {
    // digitalWrite(LED_PIN, HIGH);
    motorR.run(RELEASE);
    motorL.run(RELEASE);
    motorR.run(BACKWARD);
    motorL.run(BACKWARD);

    delay(600);
    motorR.run(RELEASE);
    motorL.run(RELEASE);

    while(1);
  }

  Serial.println(optoValue);
  //delay(5);

}
