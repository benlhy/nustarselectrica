#include "sensors.h"
#include "storage.h"
#include "pid.h"
#include <string>

//#include "Arduino"

using namespace nustars;

//HardwareSerial Serial2;

//initialize the various classes
Accelerometer* accelerometer;
Altimeter* altimeter;
GPS* gps;
PID* pid;

/**
 * Records a line to the SD card and prints it to Serial
 * @param x The input
 */
void record(String x) {
    Serial.println(x);

}

void setup() {
    //PIN SETUP
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    pinMode(A9, OUTPUT); //motor direction input A
    pinMode(A8, OUTPUT); //motor direction input B
    pinMode(A7, 0); //controls motor speed

    Serial.begin(115200); //USB
    Serial1.begin(9600); //GPS
    Serial2.begin(9600); //RADIO

    //Initialize classes
    accelerometer = new Accelerometer;
    altimeter = new Altimeter;
    gps = new GPS;
    pid = new PID;
    Serial.println("Startup complete!");
}

void loop() {
  altimeter->tick();
  accelerometer->tick();
  gps->tick();

  int x, y, z;
  x = accelerometer->getOrientation(X_AXIS);
  //y = accelerometer->getOrientation(Y_AXIS);
  //z = accelerometer->getOrientation(Z_AXIS);
  pid->tick(x);
  //Serial.println(x);
  Serial2.printf("NumSat: %d, LNG: %f\n", gps->getSat(), gps->getLng());
  //Serial2.printf("X: %3d, Y: %3d, Z: %3d\n", x, y, z);
  //radio->send("TEST");

  delay(10);
}

