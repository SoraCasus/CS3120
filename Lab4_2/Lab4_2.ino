#include <AFMotor.h>
#include <LiquidCrystal_I2C.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <math.h>

#define PI_32 3.141592654
#define BUTTON_WAIT() while(digitalRead(PUSH_BUTTON) == LOW)

#define DRIVE(x) run( (x) ? FORWARD : BACKWARD)
#define DRIVE_INV(x) run( (x) ? BACKWARD : FORWARD)

#define PUSH_BUTTON 15
#define POT_PIN A0
#define OPTO_PIN A1
#define L_COUNTER 24
#define R_COUNTER 25
#define CS_PIN 53
#define PING_PIN 46

#define DEBUG 1

uint8_t motorSpeed = 150;
uint32_t angles[] = {238, 481, 709, 942};

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);
Servo myServo;
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  lcd.init();
  lcd.backlight();

  pinMode(L_COUNTER, INPUT);
  pinMode(R_COUNTER, INPUT);
  pinMode(OPTO_PIN, INPUT);
  myServo.attach(26);

  motorR.setSpeed(motorSpeed);
  motorL.setSpeed(motorSpeed);

  motorR.run(RELEASE);
  motorL.run(RELEASE);

}

void loop() {
  // put your main code here, to run repeatedly

  uint16_t selection = analogRead(POT_PIN);
  selection = map(selection, 0, 800, 1, 8);

  //Serial.println(selection);

  //  for (int pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
  //    // in steps of 1 degree
  //    myServo.write(pos);              // tell servo to go to position in variable 'pos'
  //    delay(15);                       // waits 15ms for the servo to reach the position
  //  }
  //  for (int pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
  //    myServo.write(pos);              // tell servo to go to position in variable 'pos'
  //    delay(15);                       // waits 15ms for the servo to reach the position
  //  }

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
          lcd.print("Read Floor      ");
        }
      } break;

    case 6 : {
        if (digitalRead(PUSH_BUTTON) == LOW) {
          ReadFloor();
        } else {

          lcd.setCursor(0, 0);
          lcd.print("Read Floor     <");
          lcd.setCursor(0, 1);
          lcd.print("Sonar           ");
        }
      } break;

    case 7 : {
        if (digitalRead(PUSH_BUTTON) == LOW) {
          Sonar();
        } else {
          lcd.setCursor(0, 0);
          lcd.print("Sonar          <");
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

void Sonar() {
  BUTTON_WAIT();

  // InitSD
  pinMode(CS_PIN, OUTPUT);
  SD.begin();

  if (SD.exists("sonar.csv")) {
    SD.remove("sonar.csv");
  }

  File file = SD.open("sonar.csv", FILE_WRITE);

  // Get 3 Readings

  for (int i = 0; i < 5; i++) {

    file.print((i + 1) * 5);
    file.print(",");
    file.println(GetSonarReading());
    delay(100);
    file.print((i + 1) * 5);
    file.print(",");
    file.println(GetSonarReading());
    delay(100);
    file.print((i + 1) * 5);
    file.print(",");
    file.println(GetSonarReading());

    DriveDistance(5, false);

    delay(1000);

  }

  file.close();


}

/**
    Gets the distance using the sonar in centimeters
*/
double GetSonarReading() {
  pinMode(PING_PIN, OUTPUT);
  digitalWrite(PING_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(PING_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(PING_PIN, LOW);

  pinMode(PING_PIN, INPUT);
  int64_t duration = pulseIn(PING_PIN, HIGH);

  return ((double)duration / 29 / 2) - 11;
}

void ReadFloor() {

  BUTTON_WAIT();

  // InitSD
  pinMode(CS_PIN, OUTPUT);
  SD.begin();

  if (SD.exists("data.csv")) {
    SD.remove("data.csv");
  }

  File file = SD.open("data.csv", FILE_WRITE);
  file.println("FLOOR_SENSOR");

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

    if ((millis() - timer) % 10 == 0) {
      // Log the value to file
      uint16_t optoValue = analogRead(OPTO_PIN);
      file.println(optoValue);
    }
  }

  motorL.run(BACKWARD);
  motorR.run(BACKWARD);
  delay(100);
  motorL.run(RELEASE);
  motorR.run(RELEASE);

  file.close();

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
  DriveDistance(distance, true);

}

void GoToloc(uint16_t x, uint16_t y) {                    //////// once the required changes are made this function will work, till then nothing
  double angleR = atan(y / x);             // returns angle in radians
  uint16_t angle = angleR * 180 / PI_32;   // returns angle in degrees

  uint16_t distance = sqrt((y * y) + (x * x));


  delay(200);

  //GoToAngle(angle);           ///////// change this name or change the GoToAngle() function's name

  delay(200);

  DriveDistance(distance, true);

}

uint32_t AngleToTicks(uint16_t angle) {
  double distance = PI_32 * 2 * 16.7 * (double)angle;
  distance /= 360;
  double N = 1.04 + (5.56e-3 * angle) + (-1.11e-5 * angle * angle);

  distance /= N;
  distance *= 0.98;

  if (angle > 180) {
    distance *= 1.01;
    if (angle < 270) {
      distance *= 1.02;
    }
  }

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

  DriveDistance(distance, true);

}

/**
    Converts distance in centimeters to ticks

    distance - The distance in centimeters
    return - The distance in ticks
*/
uint32_t GetDistance(double distance) {
  if (distance <= 5) {
    return distance * 13.5;
  }
  return (uint32_t)(-55.8 + (16.2 * distance));
}

/**
    Drives the robot forward in a straight line for the
    given distance in centimeters

    distance - The distance in centimeters
*/
void DriveDistance(uint16_t distance, bool fwd) {
  bool r = LOW;
  bool l = LOW;
  bool rLast = LOW;
  bool lLast = LOW;

  motorR.DRIVE(fwd);
  motorL.DRIVE(fwd);

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
      motorR.DRIVE(fwd);
      motorL.DRIVE(fwd);
    }
  }
  motorR.DRIVE_INV(fwd);
  motorL.DRIVE_INV(fwd);
  long lastTime = millis();

  if (distance > 30) {
    while (millis() - lastTime < 200);
  } else {
    while (millis() - lastTime < 50);

  }
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
