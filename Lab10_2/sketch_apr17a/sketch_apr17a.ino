#include <AccelStepper.h>
#include <VL53L1X.h>
#include <Wire.h>

#define LAZER 1

VL53L1X lidar;

const int stepsPerRevolution = 8;
u32 reading = 20000;
int found,lost;
int alreadyFound = 0;
int shootAt, old;
int oldReading = 200000;

AccelStepper myStepper(stepsPerRevolution , 28, 29, 31, 30); 

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  pinMode(LAZER, OUTPUT);

  myStepper.setMaxSpeed(200);
  myStepper.setAcceleration(200);

  Wire.begin();

  lidar.setTimeout(500);
  lidar.init();
  lidar.setDistanceMode(VL53L1X::Short);
  lidar.setMeasurementTimingBudget(20000);

}

void loop() {
  // put your main code here, to run repeatedly:

  myStepper.moveTo(512);
  while(myStepper.currentPosition() != 511){
    myStepper.run();
  }
  myStepper.stop();

  delay(1000);
  
  lidar.startContinuous(20);

  myStepper.moveTo(-512);
  while (myStepper.currentPosition() != -511){
    
    myStepper.run();
    
    reading = lidar.read(true);

    lidar.setTimeout(0);

    Serial.println(reading);
    
    if(reading < oldReading && reading != 0 && reading > 100){
      Serial.print("          ");
      Serial.println(reading);
      shootAt = myStepper.currentPosition();
      
      oldReading = reading;
    }
  }
  Serial.println("shootAt:");
  Serial.println(shootAt);
  myStepper.stop();
  delay(1000);

  myStepper.moveTo(512);
  while(myStepper.currentPosition() != shootAt){
    myStepper.run();
  }
  myStepper.stop();
  digitalWrite(LAZER, HIGH);

  while(1);
  
}
