#include <AccelStepper.h>
#include <VL53L1X.h>
#include <Wire.h>

#define LAZER 1

VL53L1X lidar;

const int stepsPerRevolution = 8;
int reading = 0;
int found,lost;
int alreadyFound = 0;
int shootAt;

AccelStepper myStepper(stepsPerRevolution , 28, 29, 31, 30); 

void setup() {
  // put your setup code here, to run once:

  pinMode(LAZER, OUTPUT);

  myStepper.setMaxSpeed(200);
  myStepper.setAcceleration(200);

  Wire.begin();

  lidar.setTimeout(500);
  lidar.init();
  lidar.setDistanceMode(VL53L1X::Long);
  lidar.setMeasurementTimingBudget(50000);

}

void loop() {
  // put your main code here, to run repeatedly:

  myStepper.moveTo(512);
  
  lidar.startContinuous(20);
  
  while (myStepper.currentPosition() != 511){
    myStepper.run();
    reading = lidar.read(true);

    if(lidar.timeoutOccurred()) reading = 2500;

    if(reading <= 2000 && alreadyFound == 0){
      found = myStepper.currentPosition()-1;
      alreadyFound = 1;
    }
    if(alreadyFound == 1 && reading >= 2000 ){
      lost = myStepper.currentPosition()-1;
      alreadyFound = 2;
    }
  }
  myStepper.stop();
  delay(1000);

  myStepper.moveTo(-100);

  shootAt = ((found + lost)/2)+2 ;
  
  while(myStepper.currentPosition() != shootAt){
    myStepper.run();
  }
  myStepper.stop();
  digitalWrite(LAZER, HIGH);

  while(1);
  
}
