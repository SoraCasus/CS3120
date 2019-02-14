#include <AFMotor.h>
#include <LiquidCrystal_I2C.h>

#define PUSH_BUTTON 15
#define POT_PIN A0
#define OPTO_PIN 22
#define L_COUNTER 24
#define R_COUNTER 25

uint8_t motorSpeed = 150;


AF_DCMotor motorR(1);
AF_DCMotor motorL(2);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void Travel(uint16_t distance);

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  lcd.init();
  lcd.backlight();

  pinMode(L_COUNTER, INPUT);
  pinMode(R_COUNTER, INPUT);
  pinMode(OPTO_PIN, INPUT);

  motorR.setSpeed(motorSpeed);
  motorL.setSpeed(motorSpeed);

  motorR.run(RELEASE);
  motorL.run(RELEASE);

}

void loop() {
  // put your main code here, to run repeatedly

  uint16_t selection = analogRead(POT_PIN);
  selection = map(selection, 0, 1000, 1, 4);
  
  switch (selection) {
    case 1 : {
        // Go forward
        if (digitalRead(PUSH_BUTTON) == LOW) {
          MoveForward();
        } else {
          lcd.setCursor(0, 0);
          lcd.print("Go Forward     <");
          lcd.setCursor(0, 1);
          lcd.print("Count Lines     ");
        }
      } break;

    case 2 : {
        // Count Lines
        if (digitalRead(PUSH_BUTTON) == LOW) {
          CountLines();
        } else {
          lcd.setCursor(0, 0);
          lcd.print("Count Lines    <");
          lcd.setCursor(0, 1);
          lcd.print("Travel Distance ");
        }
      } break;

    case 3 : {
        // Travel Distance
        if (digitalRead(PUSH_BUTTON) == LOW) {
          TravelDistance();
        } else {
          lcd.setCursor(0, 0);
          lcd.print("Travel Distance<");
          lcd.setCursor(0, 1);
          lcd.print("Exit            ");
        }
      } break;

    case 4 : {
        // Exit
        if (digitalRead(PUSH_BUTTON) == LOW) {
          Exit();
        } else {
          lcd.setCursor(0, 0);
          lcd.print("Exit           <");
          lcd.setCursor(0, 1);
          lcd.print("                ");
        }
      } break;
  }
}

void MoveForward() {
  delay(100);
  Serial.println("BUTTON");

  uint16_t distance;

  while (digitalRead(PUSH_BUTTON) != LOW) {
    distance = analogRead(POT_PIN);
    distance = map(distance, 0, 1000, 3, 7);        // its 3 - 7 ( not  3 - 5 )

    lcd.setCursor(0, 0);
    lcd.print("Set Time        ");
    lcd.setCursor(0, 1);
    lcd.print(distance);
    lcd.print("  Seconds   ");

  }

  bool r = LOW;
  bool l = LOW;
  bool rLast = LOW;
  bool lLast = LOW;
  //motorR.setSpeed(150);
  //motorL.setSpeed(150);
  motorR.run(FORWARD);
  motorL.run(FORWARD);

  uint32_t lCounter = 0;
  uint32_t rCounter = 0;

  uint64_t timer = millis();

  while (millis() - timer < (distance * 1000)) {
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

  motorL.run(BACKWARD);
  motorR.run(BACKWARD);
  delay(100);
  motorL.run(RELEASE);
  motorR.run(RELEASE);

}

void CountLines() {
  delay(200);
  Serial.println("BUTTON");

  uint16_t distance;

  while (digitalRead(PUSH_BUTTON) != LOW) {
    distance = analogRead(POT_PIN);
    distance = map(distance, 0, 1000, 3, 5);

    lcd.setCursor(0, 0);
    lcd.print("Set Time        ");
    lcd.setCursor(0, 1);
    lcd.print(distance);
    lcd.print("  Seconds   ");

  }

  bool r = LOW;
  bool l = LOW;
  bool rLast = LOW;
  bool lLast = LOW;
  motorR.setSpeed(200);
  motorL.setSpeed(200);
  motorR.run(FORWARD);
  motorL.run(FORWARD);

  uint32_t lCounter = 0;
  uint32_t rCounter = 0;

  uint8_t lineCount = 0;
  bool lineRead = HIGH;

  uint64_t timer = millis();

  while (millis() - timer < (distance * 1000)) {
    r = digitalRead(R_COUNTER);
    // Serial.println(r);
    l = digitalRead(L_COUNTER);
    // Serial.println(l);

    bool res = digitalRead(OPTO_PIN);
    if (res != lineRead) {
      if (res == LOW) {
        lineCount++;
      }
      lineRead = res;
    }

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

  motorL.run(BACKWARD);
  motorR.run(BACKWARD);
  delay(100);
  motorL.run(RELEASE);
  motorR.run(RELEASE);

  while (digitalRead(PUSH_BUTTON) != LOW) {
    lcd.setCursor(0, 0);
    lcd.print(lineCount);
    lcd.print(" Lines        ");
    lcd.setCursor(0, 1);
    lcd.print("Press Button      ");
  }
  delay(200);
}

void TravelDistance() {
  delay(200);
  uint8_t lTicks = 0;
  uint8_t rTicks = 0;

  uint16_t distance = 0;

  while (digitalRead(PUSH_BUTTON) != LOW) {
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
  }

  Travel(distance);

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
  delay(200);

  motorR.run(RELEASE);
  motorL.run(RELEASE);
}

void Exit() {

  lcd.setCursor(0, 0);
  lcd.print("Good Bye        ");
  lcd.setCursor(0, 1);
  lcd.print("~~~~~~~~~~~~~~~");

  while(1);
}
