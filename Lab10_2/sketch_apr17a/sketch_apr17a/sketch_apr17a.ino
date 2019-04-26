#include <AFMotor.h>
#include <LiquidCrystal_I2C.h>

#define SONAR 6
#define ROT_ENCODER_CLK 46
#define ROT_ENCODER_SW 47
#define ROT_ENCODER_DT 48
#define I2C_PIN_SDA 20
#define I2C_PIN_SCL 21

int reading;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  // put your setup code here, to run once:

  
  lcd.init();
  lcd.backlight();
  
  pinMode(ROT_ENCODER_CLK, INPUT);
  pinMode(ROT_ENCODER_SW, INPUT);
  pinMode(ROT_ENCODER_DT, INPUT);
  
  motorR.setSpeed(90);
  motorL.setSpeed(110);
  
}

long getCm() {

  long duration, cm;

  pinMode(SONAR, OUTPUT);
  digitalWrite(SONAR, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR, HIGH);
  delayMicroseconds(5);
  digitalWrite(SONAR, LOW);

  pinMode(SONAR, INPUT);
  duration = pulseIn(SONAR, HIGH);

  return cm = duration / 29 / 2;

}

void loop() {
  // put your main code here, to run repeatedly:

  int distance = 15;
  bool lastEncoder = LOW;
  lcd.setCursor(0,0);
  lcd.print("SELECT DISTANCE: ");
  while (digitalRead(ROT_ENCODER_SW) != LOW)  {
    lcd.setCursor(0,1);
    lcd.print(distance);

    bool encoder = digitalRead(ROT_ENCODER_DT);
    if(encoder != lastEncoder) {
        if(encoder == HIGH){
          distance++;
          if(distance > 40) distance = 15;
          if(distance < 15) distance = 15;

          lastEncoder = encoder;
        }
        lastEncoder = encoder;
    }
  }
    

  while(1){
  reading = getCm();
  
  if(reading>

  }

}
