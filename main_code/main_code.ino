#include "sensors.h"
#include "storage.h"
#include <string>

//#include "Arduino"

using namespace nustars;

//HardwareSerial Serial2;

//initialize the various classes
Accelerometer* accelerometer;
Altimeter* altimeter;
GPS* gps;

/**
 * Records a line to the SD card and prints it to Serial
 * @param x The input
 */
void record(String x) {
    Serial.println(x);

}

void setup() {
    Serial.begin(115200); //USB
    Serial1.begin(9600); //GPS
    Serial2.begin(9600); //RADIO
    accelerometer = new Accelerometer;
    altimeter = new Altimeter;
    gps = new GPS;
    Serial.println("TEST");
}

void loop() {
  altimeter->tick();
  accelerometer->tick();
  gps->tick();

  int x, y, z;
  x = accelerometer->getOrientation(X_AXIS);
  y = accelerometer->getOrientation(Y_AXIS);
  z = accelerometer->getOrientation(Z_AXIS);
  Serial2.printf("NumSat: %d, LNG: %f\n", gps->getSat(), gps->getLng());
  //Serial2.printf("X: %3d, Y: %3d, Z: %3d\n", x, y, z);
  //radio->send("TEST");

  delay(100);
}

