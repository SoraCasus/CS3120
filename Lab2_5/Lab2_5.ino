#include <AFMotor.h>
#include <LiquidCrystal_I2C.h>

#define LASER_PIN 14
#define LED_PIN 45
#define OPTO_PIN 22

#define LEFT_SPEED 255
#define RIGHT_SPEED 255

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define L_COUNTER 24
#define R_COUNTER 25

uint32_t rCounter = 0;
uint32_t lCounter = 0;
uint32_t lastTime;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("SPEED,TIME,LEFT,RIGHT;");
  lcd.init();
  lcd.backlight();
  
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
    
    // Serial.print("TIME: "); Serial.println(_time);
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

      // motorR.run(RELEASE);
      // motorL.run(RELEASE);

      Serial.print(LEFT_SPEED);
      Serial.print(",");
      Serial.print(_time);
      Serial.print(",");
      Serial.print(lCounter);
      Serial.print(",");
      Serial.print(rCounter);
      Serial.println(";");
      
      lcd.setCursor(0, 0);
      lcd.print("Left ");
      lcd.print(lCounter);

      lcd.setCursor(0, 1);
      lcd.print("Right ");
      lcd.print(rCounter);
     }
     
    _time += 2000;
  }

  motorR.run(RELEASE);
  motorL.run(RELEASE);

  while (1);
}
