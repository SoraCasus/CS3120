#include <AFMotor.h>
#include <VL53L1X.h>
#include <LiquidCrystal_I2C.h>

#define R_ENCODER_1 14
#define L_ENCODER_1 16

#define I2C_PIN_SDA 20
#define I2C_PIN_SCL 21
#define ROT_ENCODER_CLK 46
#define ROT_ENCODER_SW 47
#define ROT_ENCODER_DT 48

#define MOTOR_SPEED 150

typedef struct pose {
  int x;
  int y;
  float angle;
} pose_t;

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);

VL53L1X lidar;

LiquidCrystal_I2C lcd(0x27, 16, 2);

pose_t currentPose;
int index;

void setup() {
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();

  Wire.begin();

  lidar.setTimeout(500);
  lidar.init();
  lidar.setDistanceMode(VL53L1X::Long);
  lidar.setMeasurementTimingBudget(50000);

  pinMode(R_ENCODER_1, INPUT);
  pinMode(L_ENCODER_1, INPUT);
  pinMode(ROT_ENCODER_CLK, INPUT);
  pinMode(ROT_ENCODER_SW, INPUT);
  pinMode(ROT_ENCODER_DT, INPUT);

  lcd.clear();

  currentPose.x = 0;
  curentPose.y = 0;
  currentPose.angle = 0;
  index = 0;

  motorR.setSpeed(MOTOR_SPEED);
  motorL.setSpeed(MOTOR_SPEED);
}

void loop() {
  lcd.clear();
  int x = 0;
  int y = 0;
  bool lastEncoder = LOW;

  // Get the X coord
  while (digitalRead(ROT_ENCODER_SW) != LOW) {
    lcd.setCursor(0, 0);
    lcd.print("X: ");
    lcd.print(x);
    lcd.print("     ");
    lcd.setCursor(0, 1);
    lcd.print("Y: ");
    lcd.print(y);
    lcd.print("    ");

    bool encoder = digitalRead(ROT_ENCODER_DT);
    if (encoder != lastEncoder) {
      if (encoder == HIGH) {
        x++;
        if (x > 200)
          x = 0;
      }
      lastEncoder = encoder;
    }
  }

  // Make sure it doesn't register as a double click
  while (digitalRead(ROT_ENCODER_SW) == LOW);

  // Get the Y coord
  while (digitalRead(ROT_ENCODER_SW) != LOW) {
    lcd.setCursor(0, 0);
    lcd.print("X: ");
    lcd.print(x);
    lcd.print("     ");
    lcd.setCursor(0, 1);
    lcd.print("Y: ");
    lcd.print(y);
    lcd.print("    ");

    bool encoder = digitalRead(ROT_ENCODER_DT);
    if (encoder != lastEncoder) {
      if (encoder == HIGH) {
        y++;
        if (y > 200)
          y = 0;
      }
      lastEncoder = encoder;
    }
  }


  float dx = x - currentPose.x;
  float dy = y - currentPose.y;

  GoToloc(dx, dy);

  currentPose.x = x;
  currentPose.y = y;

  index++;

  if (index == 3) {
    GoToloc(-currentPose.x, -currentPose.y);
    while (1);
  }

}


void GoToloc(uint16_t dx, uint16_t dy) {
  // once the required changes are made this function will work, till then nothing
  double angleR = atan2(dy, dx);             // returns angle in radians
  uint16_t angle = angleR * 180 / 3.1415926536 ;   // returns angle in degrees

  // Calculate the difference between current angle and target angle
  angle -= currentPose.angle;
  currentPose.angle += angle;

  uint16_t distance = sqrt((dy * dy) + (dx * dx));

  TurnToAngle(angle);

  delay(200);

  DriveDistance(distance);

}

/**
   Converts a given angle (in degrees) to the number of motor
   ticks needed to rotate to the given angle
*/
uint32_t AngleToTicks(uint16_t angle) {
  double distance = 3.1415926536 * 2 * 16.7 * (double)angle;
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

void TurnToAngle(uint16_t angle) {
  uint32_t targetTicks = AngleToTicks(angle);

  //for (int i = 0; i < 4; i++) {
  uint32_t lTicks = 0;
  uint32_t rTicks = 0;

  bool r = LOW;
  bool rLast = LOW;
  bool l = LOW;
  bool lLast = LOW;

  bool positive = angle > 0;
  if (positive) {
    motorR.setSpeed(150);
    motorR.run(FORWARD);
    motorL.setSpeed(150);
    motorL.run(BACKWARD);
  } else {
    angle *= -1;
    motorR.setSpeed(150);
    motorR.run(BACKWARD);
    motorL.setSpeed(150);
    motorL.run(FORWARD);
    
  }
  while (lTicks < targetTicks || rTicks < targetTicks) {
    r = digitalRead(R_ENCODER_1);
    l = digitalRead(L_ENCODER_1);

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

    r = digitalRead(R_ENCODER_1);
    // Serial.println(r);
    l = digitalRead(L_ENCODER_1);
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

  if (distance > 30) {
    while (millis() - lastTime < 200);
  } else {
    while (millis() - lastTime < 50);

  }
  motorR.run(RELEASE);
  motorL.run(RELEASE);
}
