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
bool automaticLaunchDetected = false;
int manualFlywheelOverride = 0; // -1: force off, 0: don't care, 1: force on

//initialize the various classes
Accelerometer* accelerometer = NULL;
Altimeter* altimeter = NULL;
GPS* gps = NULL;
PID* pid= NULL;
Radio* radio = NULL;
Storage* storage = NULL;

long lastLoopTime = 0;
long lastBroadcast = 0;

const int LED_PINS[] = {24, 25, 26};
const char FILE_NAME[] = "out.txt";
unsigned int P = 1000;
unsigned int I = 0;
unsigned int D = 0;

void setup() {
    pinMode(24, OUTPUT);
    pinMode(25, OUTPUT);
    pinMode(26, OUTPUT);

    //PIN SETUP
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    pinMode(A9, OUTPUT); //motor direction input A
    pinMode(A8, OUTPUT); //motor direction input B
    pinMode(A7, 0); //controls motor speed
    
    Serial.begin(9600); //USB
    delay(1000);
    Serial1.begin(9600); //GPS
    Serial2.begin(9600); //RADIO
    //Initialize classes
    accelerometer = new Accelerometer;
    altimeter = new Altimeter;
    gps = new GPS;
    pid = new PID;
    radio = new Radio;
    storage = new Storage("out.txt");
    //Serial.println("Startup complete!");
}

void loop() {
  altimeter->tick();
  accelerometer->tick();
  gps->tick();

  int x_rot, y_rot, z_rot;

  x_rot = accelerometer->getOrientation(X_AXIS);
  y_rot = accelerometer->getOrientation(Y_AXIS);
  z_rot = accelerometer->getOrientation(Z_AXIS);
  //if (automaticLaunchDetected) {
      pid->tick(x_rot);
  //}
  long thisTime = millis();

  //do everything involving radio broadcast
  if (thisTime - lastBroadcast > BROADCAST_DELAY) { //so we don't kill the radios
      char* msg = new char[100];
      sprintf(msg, "NU T:%lu/X:%f/Y:%d/Z:%d/Tr:%d/Ln:%.2f/Lt:%.2f/Lp:%lu/A:%d/\n", thisTime, pid->getP(), y_rot, z_rot, pid->getDesiredX(), gps->getLng(), gps->getLat(), (thisTime - lastLoopTime), altimeter->getAltitude());
      //Serial.println(msg);
      Serial2.printf(msg);
      storage->write(msg);
      delete msg;
  }

  //do everything involving the radio reciever
  if (Serial2.available() >= 20) {
      char* s = new char[20]{0};
      for (int i = 0; i < 20; i++) {
          s[i] = Serial2.read();
      }
      Serial2.flush();
      bool pb = false, ib = false, db = false;
      for (int j = 0; j < 20 - 2; j++) {
          if (s[j] == 'N' && s[j+1] == 'U') {
              for (int i = 0; i < 20 - 2; i++) {
                  if (s[i] == 'P' && s[i + 1] == '/') {
                      P = 0;
                      for (int n = 0; n < 2; n++) {
                          P = (P << 8) + s[i + 2 + n];
                          pb = true;
                      }
                  } else if (s[i] == 'I' && s[i + 1] == '/') {
                      I = 0;
                      for (int n = 0; n < 2; n++) {
                          I = (I << 8) + s[i + 2 + n];
                          ib = true;
                      }
                  } else if (s[i] == 'D' && s[i + 1] == '/') {
                      D = 0;
                      for (int n = 0; n < 2; n++) {
                          D = (D << 8) + s[i + 2 + n];
                          db = true;
                      }
                  }
                  if (pb && ib && db) {
                      break;
                  }
              }
              break;
          }
      }
      pid->setPID(P/1000.0, I/1000.0, D/1000.0);
      delete[] s;
  }

  //handle the launch detection
  //yes this is 39 on purpose it does not go higher this is so sad
  if (accelerometer->getAcceleration(Y_AXIS) == 39 || accelerometer->getAcceleration(Y_AXIS) == -39) {
      automaticLaunchDetected = true;
  }
  //Serial.println(thisTime - lastLoopTime);
  lastLoopTime = thisTime;
}

