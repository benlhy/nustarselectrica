//  My First Rocket
//#include "display_header.h"
//#include "radio_functions.h"
//#include "MPU6050/MPU6050.h"
//#include "pid.h"
// This code assumes the usage of the following sensors:
// BMP280 - altitude
// BNO055 - gyro
// Adafruit ULTIMATE GPS
// Motor control from Polulu

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <utility/imumaths.h>

#define mySerial Serial1




/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true

Adafruit_BMP280 bme; // I2C

Adafruit_BNO055 bno = Adafruit_BNO055(55);


int prevE = 0;
int prevEI = 0;
int prevED = 0;
int currE = 0;
int controlu = 0;


int orientX = 0;
int orientY = 0;
int orientZ = 0;

int temp = 0;
int pressure = 0;
int alt = 0;

float desiredX = 0;
float desiredY = 0;
float desiredZ = 0;

int Kp = 0, Kd = 0, Ki = 0;

float lat=0,lon=0;
uint32_t timer = millis();


boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
/////////////////////////// GPS INIT/////////////////////////
void gps_init() {
  GPS.begin(9600);
  mySerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
#ifdef __arm__
  usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
#else
  useInterrupt(true);
#endif

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}

#ifdef __AVR__
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif //#ifdef__AVR__


void gps_update(){
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  
    if (timer > millis())  timer = millis();
  
    // approximately every 2 seconds or so, print out the current stats
    if (millis() - timer > 2000) { 
     
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      lat = GPS.lat;
      lon = GPS.lon;
    }
    else{
      Serial.print("No fix!");
      lat = 0;
      lon = 0;
    }
    }
  
  
}




/////////////////////////// SENSOR TEAM   /////////////////////
void sensor_init() {
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  if (!bme.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

}


void sensor_update() {
  sensors_event_t event;
  bno.getEvent(&event);
  float collectX = 0;
  float collectY = 0;
  float collectZ = 0;
  // The data will be very noisy, so we have to apply a moving average
  for (int i = 0; i < 10; i++) {
    sensors_event_t event;
    bno.getEvent(&event);
    collectX = event.orientation.x + collectX;
    collectY = event.orientation.y + collectY;
    collectZ = event.orientation.z + collectZ;
    delay(5);
  }
  orientX = collectX / 10;
  orientY = collectY / 10;
  orientZ = collectZ / 10;
  temp = bme.readTemperature();
  pressure = bme.readPressure();
  alt = bme.readAltitude(997.03);
  Serial.print("Temperature = ");
  Serial.print(temp);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" Pa");

  Serial.print("Approx altitude = ");
  Serial.print(alt); // this should be adjusted to your local forcase
  Serial.println(" m");
}
////////////////////////// MOTOR TEAM /////////////////////////
void motor_init() {
  pinMode(A9, OUTPUT); //motor direction input A
  pinMode(A8, OUTPUT); //motor direction input B
  pinMode(A7, 0); //controls motor speed
}
// update the pwm signal based on input from sensors
void motor_update () {

  currE = desiredX - orientX;
  prevED = prevE - currE;
  prevEI = prevEI + currE;
  // anti integrator
  if (prevEI > 60) {
    prevEI = 60;
  }
  if (prevEI < -60) {
    prevEI = -60;
  }
  controlu = Kp * currE + Kd * prevED + Ki * prevE;
  prevE = currE;
  if (controlu > 255) {
    digitalWrite(A8, 1);
    digitalWrite(A9, 0);
    analogWrite(A7, 255);
  }
  else if ((controlu < 0) && (controlu > -255)) {
    // if we have a negative controlu
    digitalWrite(A8, 0);
    digitalWrite(A9, 1);
    analogWrite(A7, -controlu); //make it positive
  }
  else if (controlu < -255) {
    digitalWrite(A8, 0);
    digitalWrite(A9, 1);
    analogWrite(A7, 255);
  }
  else if (controlu > 0 && controlu < 255) {
    digitalWrite(A8, 1);
    digitalWrite(A9, 0);
    analogWrite(A7, controlu);
  }
  else if (controlu == 0) {
    digitalWrite(A8, 0);
    digitalWrite(A9, 0);
    analogWrite(A7, controlu);
  }
}



///////////////////////////////////////////////////////////////

///////////////////////// RADIO TEAM //////////////////////////
void radio_init() {
  Serial2.begin(9600); // USE TX/RX 2 of Teensy 3.5
  for (int i = 0; i < 10; i++) {
    Serial2.write("Radio check");
  }
}

void radio_update() {

  char reply[20];
  Serial2.write("Ground Station: ");
  Serial.print("Sending val: ");
  sprintf(reply, "X:%d Y:%d Z:%d ", orientX, orientY, orientZ);
  Serial2.write(reply);
  Serial.print(reply);
  char reply2[20];
  sprintf(reply2, "A:%d P:%d T:%d ", alt, pressure, temp);
  Serial2.write(reply2);
  Serial.print(reply2);
  char reply3[30];
  sprintf(reply3, "LA:%f LO:%f", lat, lon);
  Serial2.write(reply3);
  Serial.println(reply3);

}
///////////////////////////////////////////////////////////////

//////////////////////////CAMERA CODE//////////////////////////
/*
int trig = 0;
int led = 1;

void camera_setup() {
  // initialize digital pins as output
  pinMode(led, OUTPUT);
  pinMode(trig, OUTPUT);

  digitalWrite(led, HIGH);
  digitalWrite(trig, HIGH);

}*/
///////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////
void setup() {

  Serial.begin(115200);
  delay(1000);
  Serial.println("Hello");
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  gps_init();

  //radio_init();
  //camera_setup();
  //sensor_init();
  //motor_init();
  

}






void loop() {


  // Here we update the sensors
  //sensor_update();// delay for 50ms
  // Based on the sensors, pass sensor data to motors
  //motor_update();
  // update groundstation
  //radio_update();
  gps_update();
  //delay(500);

}



