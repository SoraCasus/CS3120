#include <AFMotor.h>


// Note: Using defines for constants to preserve memory usage
#define LASER_PIN 14
#define LED_PIN 21
#define OPTO_PIN 22

#define LEFT_SPEED 80
#define RIGHT_SPEED 60

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);

bool foundLine = false;

void setup() {
  Serial.begin(9600);

  pinMode(LASER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(OPTO_PIN, INPUT);

  digitalWrite(LASER_PIN, HIGH);
  digitalWrite(LED_PIN, LOW);

  // Give a burst of speed to make sure it actually moves
  motorR.setSpeed(200);
  motorL.setSpeed(200);
  motorR.run(FORWARD);
  motorL.run(FORWARD);

  delay(50);
}

void loop() {
  int32_t optoValue = digitalRead(OPTO_PIN);
  Serial.println(optoValue);

  if(!foundLine) {
    // Search for the line
    motorR.setSpeed(RIGHT_SPEED);
    motorL.setSpeed(LEFT_SPEED);
    motorR.run(FORWARD);
    delay(50);
    motorL.run(FORWARD);

    if(optoValue == LOW) {
      // Found the line!
      foundLine = true;
      motorR.run(RELEASE);
      motorL.run(RELEASE);
      delay(200);
    }
  } else {
    // Back up until over line
    motorR.setSpeed(RIGHT_SPEED);
    motorL.setSpeed(LEFT_SPEED);
    motorR.run(BACKWARD);
    motorL.run(BACKWARD);

    while(optoValue == LOW) {   
     optoValue = digitalRead(OPTO_PIN);
    //  if(optoValue == HIGH) break;
      
      Serial.println(optoValue);
      motorR.run(RELEASE);
      motorL.run(RELEASE);
      digitalWrite(LED_PIN, HIGH);
    }
    
      digitalWrite(LED_PIN, LOW);
  }
  
   
  delay(1);
}
