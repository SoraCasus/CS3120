#include <AFMotor.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define PUSH_PIN 15
#define POT_PIN A0
#define L_COUNTER 24
#define R_COUNTER 25

uint8_t motorSpeed = 150;
byte block[8] = {
  0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff
};

void Travel(uint16_t distance);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();

  pinMode(L_COUNTER, INPUT);
  pinMode(R_COUNTER, INPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(PUSH_PIN, INPUT);

  motorR.setSpeed(motorSpeed);
  motorL.setSpeed(motorSpeed);
  motorR.run(RELEASE);
  motorL.run(RELEASE);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t lTicks = 0;
  uint8_t rTicks = 0;

  uint16_t distance = 0;


  distance = analogRead(POT_PIN);
  distance = map(distance, 0, 1023, 20, 120);
  lcd.setCursor(0, 0);
  lcd.print("Distance: ");
  lcd.setCursor(10, 0);
  lcd.print(distance);
  lcd.print("   ");
  lcd.setCursor(0, 1);

  uint8_t d = map(distance, 20, 120, 1, 16);
  for (int i = 0; i < 16; i++)  {
    if (i < d)
      lcd.print("@");
    else
      lcd.print(" ");
  }

  if (digitalRead(PUSH_PIN) == LOW) {
    Travel(distance);
  }
}

uint32_t GetDistance(uint16_t distance) {
  return (uint32_t)(-55.8 + (16.2 * distance));
}

void Travel(uint16_t distance) {
  bool r = LOW;
  bool l = LOW;
  bool rLast = LOW;
  bool lLast = LOW;

  motorR.run(FORWARD);
  motorL.run(FORWARD);

  uint32_t target = GetDistance(distance);
  uint32_t lCounter = 0;
  uint32_t rCounter = 0;

  while (lCounter < target && rCounter < target) {

    r = digitalRead(R_COUNTER);
    // Serial.println(r);
    l = digitalRead(L_COUNTER);
    // Serial.println(l);

    if (r != rLast) {
      if (r == HIGH)   {
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

    if (lCounter > rCounter) {
      motorL.run(RELEASE);
    } else if (rCounter > lCounter) {
      motorR.run(RELEASE);
    } else {
      motorR.run(FORWARD);
      motorL.run(FORWARD);
    }

  }

  motorR.run(BACKWARD);
  motorL.run(BACKWARD);
  delay(50);

  motorR.run(RELEASE);
  motorL.run(RELEASE);
}
