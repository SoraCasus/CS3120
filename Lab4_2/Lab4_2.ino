#include <AFMotor.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

#define PI_32 3.141592654
#define BUTTON_WAIT() while(digitalRead(PUSH_BUTTON) == LOW)

#define PUSH_BUTTON 15
#define POT_PIN A0
#define OPTO_PIN 22
#define L_COUNTER 24
#define R_COUNTER 25

#define DEBUG 1

uint8_t motorSpeed = 150;

uint32_t angles[] = {238, 481, 709, 942};

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);
LiquidCrystal_I2C lcd(0x27, 16, 2);


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

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
  selection = map(selection, 0, 800, 1, 7);

  //Serial.println(selection);

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
          lcd.print("Go to angle     ");
        }
      } break;

    case 4 : {
        if (digitalRead(PUSH_BUTTON) == LOW) {
          GoToAngle();
        } else {
          lcd.setCursor(0, 0);
          lcd.print("Go to angle    <");
          lcd.setCursor(0, 1);
          lcd.print("Go to goal      ");
        }
      } break;

    case 5 : {
        if (digitalRead(PUSH_BUTTON) == LOW) {
          GoToGoal();
        } else {
          lcd.setCursor(0, 0);
          lcd.print("Go to goal     <");
          lcd.setCursor(0, 1);
          lcd.print("Rot Calibrate   ");
        }
      } break;

    case 6 : {
        if (digitalRead(PUSH_BUTTON) == LOW) {
          CalibrateRotation();
        } else {

          lcd.setCursor(0, 0);
          lcd.print("Rot Calibrate  <");
          lcd.setCursor(0, 1);
          lcd.print("Exit            ");
        }
      } break;

    default : {
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
  delay(200);
}

void CalibrateRotation() {
  uint32_t targetTicks = 3000;

  //for (int i = 0; i < 4; i++) {
  uint32_t lTicks = 0;
  uint32_t rTicks = 0;

  bool r = LOW;
  bool rLast = LOW;
  bool l = LOW;
  bool lLast = LOW;

  int index = 0;
  bool optoLast = LOW;
  bool opto = LOW;
  

  motorR.setSpeed(150);
  motorR.run(FORWARD);
  motorL.setSpeed(150);
  motorL.run(BACKWARD);

  int i = 0;

  while (lTicks < targetTicks || rTicks < targetTicks) {
    r = digitalRead(R_COUNTER);
    l = digitalRead(L_COUNTER);

//    lcd.setCursor(0, 0);
//    lcd.print(lTicks);
//    lcd.print("          ");
//    lcd.setCursor(0, 1);
//    lcd.print(rTicks);
//    lcd.print("          ");

    if (lTicks >= targetTicks) {
      motorL.run(RELEASE);
    }

    if (rTicks >= targetTicks) {
      motorR.run(RELEASE);
    }

    if (r != rLast) {
      if (r == HIGH) {
        rTicks++;
      }
      rLast = r;
    }

    if (l != lLast) {
      if (l == HIGH) {
        lTicks++;
      }
      lLast = l;
    }

    if (lTicks > rTicks) {
      motorL.run(RELEASE);
    } else {
      motorL.run(BACKWARD);
    }

    if (rTicks > lTicks) {
      motorR.run(RELEASE);
    } else {
      motorR.run(FORWARD);
    }

    opto = digitalRead(OPTO_PIN);
    Serial.println(opto);
    if(opto != optoLast) {
      if(opto == LOW) {
        if(i++ < 10) continue;
        angles[index++] = lTicks;
        if(index > 3) break;
      }
      optoLast = opto;
    }
    
  }

  long lastTime = millis();


  while ((millis() - lastTime) < 200) {
    motorR.run(BACKWARD);
    motorL.run(FORWARD);
  }


  motorR.run(RELEASE);
  motorL.run(RELEASE);

  lastTime = millis();
  while ((millis() - lastTime) < 1000);

  Serial.print("90: ");
  Serial.println(angles[0]);
  Serial.print("180: ");
  Serial.println(angles[1]);
  Serial.print("270: ");
  Serial.println(angles[2]);
  Serial.print("360: ");
  Serial.println(angles[3]);
  

}

void GoToGoal() {
  BUTTON_WAIT();

  lcd.setCursor(0, 0);
  lcd.print("Angle: ");

  uint16_t angle = 0;

  uint16_t distance = 0;

  // Retrieve the Angle
  while (digitalRead(PUSH_BUTTON) != LOW) {
    angle = analogRead(POT_PIN);
    angle = map(angle, 0, 1023, 0, 72);
    angle *= 5;

    lcd.setCursor(6, 0);
    lcd.print(angle);
    lcd.print("   ");
  }

  BUTTON_WAIT();

  lcd.setCursor(0, 1);
  lcd.print("Distance: ");

  // Retrieve the Distance
  while (digitalRead(PUSH_BUTTON) != LOW) {
    distance = analogRead(POT_PIN);
    distance = map(distance, 0, 1023, 20, 200);

    lcd.setCursor(9, 1);
    lcd.print(distance);
    lcd.print("   ");
  }



  TurnToAngle(angle);
  DriveDistance(distance);

}

void GoToloc(uint16_t x, uint16_t y) {                    //////// once the required changes are made this function will work, till then nothing
  double angleR = atan(y / x);             // returns angle in radians
  uint16_t angle = angleR * 180 / PI_32;   // returns angle in degrees

  uint16_t distance = sqrt((y * y) + (x * x));


  delay(200);

  //GoToAngle(angle);           ///////// change this name or change the GoToAngle() function's name

  delay(200);

  DriveDistance(distance);

}

uint32_t AngleToTicks(uint16_t angle) {
  double distance = PI_32 * 2 * 16.7 * (double)angle;
  distance /= 360;
  double N = 1.04 + (5.56e-3 * angle) + (-1.11e-5 * angle * angle);

   distance /= N;
  distance *= 0.98;

  return GetDistance(distance);
}

void GoToAngle() {
  BUTTON_WAIT();

  uint16_t angle = 0;
#if DEBUG
  Serial.println("In angle loop");
#endif
  while (digitalRead(PUSH_BUTTON) != LOW) {
    angle = analogRead(POT_PIN);
    angle = map(angle, 0, 1023, 0, 72);
    angle *= 5;

    lcd.setCursor(0, 0);
    lcd.print("Set Angle    ");
    lcd.print(angle);
    lcd.print("   ");
    lcd.setCursor(0, 1);
    lcd.print("                ");

  }

  BUTTON_WAIT();

  TurnToAngle(angle);
}

void TurnToAngle(uint16_t angle) {
  uint32_t targetTicks = AngleToTicks(angle);

  //for (int i = 0; i < 4; i++) {
  uint32_t lTicks = 0;
  uint32_t rTicks = 0;

  bool r = LOW;
  bool rLast = LOW;
  bool l = LOW;
  bool lLast = LOW;

  motorR.setSpeed(150);
  motorR.run(FORWARD);
  motorL.setSpeed(150);
  motorL.run(BACKWARD);

  while (lTicks < targetTicks || rTicks < targetTicks) {
    r = digitalRead(R_COUNTER);
    l = digitalRead(L_COUNTER);

    if (lTicks >= targetTicks) {
      motorL.run(RELEASE);
    }

    if (rTicks >= targetTicks) {
      motorR.run(RELEASE);
    }

    if (r != rLast) {
      if (r == HIGH) {
        rTicks++;
      }
      rLast = r;
    }

    if (l != lLast) {
      if (l == HIGH) {
        lTicks++;
      }
      lLast = l;
    }

    if (lTicks > rTicks) {
      motorL.run(RELEASE);
    } else {
      motorL.run(BACKWARD);
    }

    if (rTicks > lTicks) {
      motorR.run(RELEASE);
    } else {
      motorR.run(FORWARD);
    }
  }

#if DEBUG
  Serial.println("Ticks : ");
  Serial.print("Left_Tick: ");
  Serial.print(lTicks);
  Serial.print("       Right_Tick: ");
  Serial.print(rTicks);
  Serial.println();

  Serial.println("Done Turning");
#endif

  long lastTime = millis();


  if (angle < 90) {
    while ((millis() - lastTime) < 100) {
      motorR.run(BACKWARD);
      motorL.run(FORWARD);
    }
  } else if (angle < 270) {
    while ((millis() - lastTime) < 160) {
      motorR.run(BACKWARD);
      motorL.run(FORWARD);
    }
  } else {
    while ((millis() - lastTime) < 200) {
      motorR.run(BACKWARD);
      motorL.run(FORWARD);
    }
  }

  motorR.run(RELEASE);
  motorL.run(RELEASE);

  lastTime = millis();
  while ((millis() - lastTime) < 1000);
}

void MoveForward() {
  BUTTON_WAIT();
  Serial.println("BUTTON");

  uint16_t distance;

  while (digitalRead(PUSH_BUTTON) != LOW) {
    distance = analogRead(POT_PIN);
    distance = map(distance, 0, 1000, 3, 7);        // its 3 - 7 ( not  3 - 5 )

    lcd.setCursor(0, 0);
    lcd.print("Set Time        ");
    lcd.setCursor(0, 1);
    lcd.print(distance);
    lcd.print("  Seconds       ");

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
  BUTTON_WAIT();
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
  BUTTON_WAIT();
}

/**
   Retrieves the distance to drive from the user
   the drives the bot forward in the specified distance
*/
void TravelDistance() {
  BUTTON_WAIT();
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

  DriveDistance(distance);

}

/**
    Converts distance in centimeters to ticks

    distance - The distance in centimeters
    return - The distance in ticks
*/
uint32_t GetDistance(double distance) {
  if (distance <= 5) {
    return distance * 12;
  }
  return (uint32_t)(-55.8 + (16.2 * distance));
}

/**
    Drives the robot forward in a straight line for the
    given distance in centimeters

    distance - The distance in centimeters
*/
void DriveDistance(uint16_t distance) {
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
  long lastTime = millis();
  while(millis() - lastTime < 200);

  motorR.run(RELEASE);
  motorL.run(RELEASE);
}

void Exit() {

  lcd.setCursor(0, 0);
  lcd.print("Good Bye        ");
  lcd.setCursor(0, 1);
  lcd.print("~~~~~~~~~~~~~~~~");

  while (1);
}
