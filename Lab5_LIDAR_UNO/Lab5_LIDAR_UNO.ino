#include <VL53L1X.h>
#include <Wire.h>

VL53L1X sensor;

int targetDistance = 5000;
byte targetReached = 0;

void setup() {
  
  Wire.begin(8);

  Wire.onReceive(ReceiveEvent);
  Wire.onRequest(RequestEvent);

  sensor.setTimeout(500);
  sensor.init();

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  sensor.startContinuous(50);
}

void ReceiveEvent(int numBytes) {
  byte lowOrder = Wire.read();
  byte highOrder = Wire.read();

  targetDistance = (highOrder << 8) | lowOrder;
}

void RequestEvent() {
  Wire.write(targetReached);
}

void loop() {
  int sensorReading = sensor.read(true);
  // Don't check if target has been reached if the TOF sensor
  // has timed out.
  if(sensor.timeoutOccurred()) return;

  sensorReading /= 10;
  if(sensorReading <= targetDistance && sensorReading > 10) targetReached = 1;
}
