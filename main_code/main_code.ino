//#include "sensors.h"
//#include "storage.h"
//#include "pid.h"
//#include "radio.h"
//#include <string>

//#include "Arduino"

//using namespace nustars;

//Constants
const long BROADCAST_DELAY = 0;

//Evil globals
bool trackingIsOn = false;

//initialize the various classes
//Accelerometer* accelerometer = NULL;
//Altimeter* altimeter = NULL;
//GPS* gps = NULL;
//PID* pid= NULL;
//Radio* radio = NULL;

long lastLoopTime = 0;
long lastBroadcast = 0;

const int LED_PINS[] = {24, 25, 26};
int P = 0;
int I = 0;
int D = 0;
void setup() {
    /*
    pinMode(24, OUTPUT);
    pinMode(25, OUTPUT);
    pinMode(26, OUTPUT);

    //PIN SETUP
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    pinMode(A9, OUTPUT); //motor direction input A
    pinMode(A8, OUTPUT); //motor direction input B
    pinMode(A7, 0); //controls motor speed
    */
    Serial.begin(9600); //USB
    /*
    Serial1.begin(9600); //GPS
    Serial2.begin(9600); //RADIO
    */
    //Initialize classes
    /*accelerometer = new Accelerometer;
    altimeter = new Altimeter;
    gps = new GPS;
    pid = new PID;
    radio = new Radio;*/
    Serial.println("Startup complete!");
}

void loop() {
  Serial.println("Looping");
    /*
  digitalWrite(24, HIGH);
  digitalWrite(25, LOW);
  digitalWrite(26, LOW);*/
  //altimeter->tick();
  //accelerometer->tick();
  //gps->tick();

  int x, y, z;
  //x = accelerometer->getOrientation(X_AXIS);
  //pid->tick(x);
  //Serial2.printf("NumSat: %d, LNG: %f\n", gps->getSat(), gps->getLng());
    /*
  long thisTime = millis();
  if (thisTime - lastBroadcast > BROADCAST_DELAY) { //so we don't kill the radios
      Serial2.printf("T:%d/X:%d/Tr:%d/Ln:%.2f/Lt:%.2f/\n", thisTime, x, pid->getP(), gps->getLng(), gps->getLat());
  }

  if (Serial2.available() > 0) {
      String s = Serial2.readString();
      bool pb = false, ib = false, db = false;
      for (int i = 0; i < s.length() - 2; i++) {
          if (s[i] == 'P' && s[i + 1] == '/') {
              P = 0;
              for (int n = 0; n < 4; n++) {
                  P = (P << 8) + s[i + 2 + n];
                  pb = true;
              }
              } else if (s[i] == 'I' && s[i + 1] == '/') {
                  I = s[i + 2];
                  ib = true;
              } else if (s[i] == 'D' && s[i + 1] == '/') {
                  D = s[i + 2];
                  db = true;
              }
              if (pb && ib && db) {
                  break;
              }

      }
      pid->setPID(P/1000.0, I/1000.0, D/1000.0);
      Serial.println(s);
  }
 */
  //Serial.println(thisTime - lastLoopTime);
  //lastLoopTime = thisTime;
}

