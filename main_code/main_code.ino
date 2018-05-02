#include "sensors.h"
#include "radio.h"
#include <string>
//#include "Arduino"

using namespace nustars;

//initialize the various classes
Accelerometer* accelerometer;
Altimeter* altimeter;
Xbee* radio;

void setup() {
    Serial.begin(115200);
    accelerometer = new Accelerometer;
    altimeter = new Altimeter;
    radio = new Xbee;
    Serial.println("TEST");
}

void loop() {
  altimeter->tick();
  accelerometer->tick();
  int x, y, z;
  x = accelerometer->getOrientation(X_AXIS);
  y = accelerometer->getOrientation(Y_AXIS);
  z = accelerometer->getOrientation(Z_AXIS);
  Serial.printf("X: %3d, Y: %3d, Z: %3d\n", x, y, z);
  radio->send();
  delay(100);
}

