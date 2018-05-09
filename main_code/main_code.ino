#include "sensors.h"
#include "storage.h"
#include "pid.h"
#include "radio.h"
#include <string>

//#include "Arduino"

using namespace nustars;

//Constants
const long BROADCAST_DELAY = 0;

//Evil globals
bool trackingIsOn = false;

//initialize the various classes
Accelerometer* accelerometer;
Altimeter* altimeter;
GPS* gps;
PID* pid;
Radio* radio;

long lastLoopTime = 0;
long lastBroadcast = 0;

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
    radio = new Radio;
    Serial.println("Startup complete!");
}

void loop() {
  altimeter->tick();
  accelerometer->tick();
  gps->tick();

  int x, y, z;
  x = accelerometer->getOrientation(X_AXIS);
  pid->tick(x);
  //Serial2.printf("NumSat: %d, LNG: %f\n", gps->getSat(), gps->getLng());


  long thisTime = millis();
  if (thisTime - lastBroadcast > BROADCAST_DELAY) { //so we don't kill the radios
      Serial2.printf("T:%d/X:%d/Tr:%d/Ln:%.2f/Lt%.2f/\n", thisTime, x, pid->getDesiredX(), gps->getLng(), gps->getLat());
  }
  Serial.println(thisTime - lastLoopTime);
  lastLoopTime = thisTime;
}

