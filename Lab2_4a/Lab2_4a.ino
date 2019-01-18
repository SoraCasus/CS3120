#include <AFMotor.h>

#define LASER_PIN 14
#define LED_PIN 45
#define OPTO_PIN 22

#define LEFT_SPEED 100
#define RIGHT_SPEED 70

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);

uint8_t count = 0;
uint32_t lastTime;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  lastTime = millis();


  pinMode(LASER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(OPTO_PIN, INPUT);

  digitalWrite(LASER_PIN, HIGH);
  digitalWrite(LED_PIN, LOW);

  motorR.setSpeed(RIGHT_SPEED);
  motorL.setSpeed(LEFT_SPEED);
}

void loop() {
  // put your main code here, to run repeatedly:

  motorR.run(FORWARD);
  motorL.run(FORWARD);

  bool lastResult = HIGH;
  while(millis() - lastTime < 5000) {
    // Counting lines
    bool res = digitalRead(OPTO_PIN);
    if(res != lastResult) {
      if(res == LOW) {
        // increment the counter
        count++;
      }
      lastResult = res;
    }
  Serial.println(digitalRead(OPTO_PIN));
  }

  motorR.run(BACKWARD);
  motorL.run(BACKWARD);
  delay(160);
  motorR.run(RELEASE);
  motorL.run(RELEASE);
  
  Serial.println(digitalRead(OPTO_PIN));

  delay(2000);
  // Done counting
  for(int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);  
  }

  while(1);
  
}
