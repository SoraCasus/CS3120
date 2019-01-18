#include <AFMotor.h>

#define LASER_PIN 14
#define LED_PIN 45
#define OPTO_PIN 22

#define LEFT_SPEED 75
#define RIGHT_SPEED 75

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);

#define L_COUNTER 24
#define R_COUNTER 25

uint32_t rCounter = 0;
uint32_t lCounter = 0;
uint32_t lastTime;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  pinMode(OPTO_PIN, INPUT);
  pinMode(LASER_PIN, OUTPUT);

  // MotorCounters
  pinMode(L_COUNTER, INPUT);
  pinMode(R_COUNTER, INPUT);

  motorR.setSpeed(RIGHT_SPEED);
  motorL.setSpeed(LEFT_SPEED);

  motorR.run(FORWARD);
  motorL.run(FORWARD);

  lastTime = millis();

}

void loop() {
  // put your main code here, to run repeatedly:
  bool r = LOW;
  bool l = LOW;
  bool rLast = LOW;
  bool lLast = LOW;

  rCounter = 0;


  motorR.run(FORWARD);
  motorL.run(FORWARD);
  delay(500);
  uint32_t _time = 2000;
  for (int j = 0; j < 5; j++) {
    
    Serial.print("TIME: "); Serial.println(_time);
    for (int i = 0; i < 5; i++) {
      rCounter = 0;
      lCounter = 0;

      motorR.run(FORWARD);
      motorL.run(FORWARD);
      lastTime = millis();
      while (millis() - lastTime < _time) {
        r = digitalRead(R_COUNTER);
        l = digitalRead(L_COUNTER);

        if (r != rLast) {
          if (r == HIGH) {
            rCounter++;
          }
          rLast = r;
        }


        if (l != lLast) {
          if (l == HIGH) {
            lCounter++;
          }
          lLast = l;
        }
      }

      motorR.run(RELEASE);
      motorL.run(RELEASE);

      Serial.print("Left Wheel Ticks: ");
      Serial.println(lCounter);

      Serial.print("Right Wheel Ticks: ");
      Serial.println(rCounter);
    }
    _time += 2000;
  }

  while (1);
}
