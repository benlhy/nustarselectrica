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
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define mySerial Serial1

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true

Adafruit_BMP280 bme; // I2C

Adafruit_BNO055 bno = Adafruit_BNO055(55);

TinyGPSPlus gps; // define tinygps as gps


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

float lat = 0, lon = 0;
uint32_t timer = millis();


boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
/////////////////////////// GPS INIT/////////////////////////




static void smartdelay(unsigned long ms);

void gps_init(){
  mySerial.begin(9600); //
}


void gps_update() {
  float flat, flon; // variable definition
  int year; // variable definition for age
  byte month, day, hour, minute, second, hundredths;
  unsigned long age; // variable definition for age

  while (mySerial.available() > 0)
    gps.encode(mySerial.read());

  //if (gps.altitude.isUpdated())
  //  Serial.println(gps.altitude.meters());

  Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
  Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
  Serial.print("ALT=");  Serial.println(gps.altitude.meters());

  /*
    Serial.println(gps.location.lat(), 6); // Latitude in degrees (double)
    Serial.println(gps.location.lng(), 6); // Longitude in degrees (double)
    Serial.print(gps.location.rawLat().negative ? "-" : "+");
    Serial.println(gps.location.rawLat().deg); // Raw latitude in whole degrees
    Serial.println(gps.location.rawLat().billionths);// ... and billionths (u16/u32)
    Serial.print(gps.location.rawLng().negative ? "-" : "+");
    Serial.println(gps.location.rawLng().deg); // Raw longitude in whole degrees
    Serial.println(gps.location.rawLng().billionths);// ... and billionths (u16/u32)
    Serial.println(gps.date.value()); // Raw date in DDMMYY format (u32)
    Serial.println(gps.date.year()); // Year (2000+) (u16)
    Serial.println(gps.date.month()); // Month (1-12) (u8)
    Serial.println(gps.date.day()); // Day (1-31) (u8)
    Serial.println(gps.time.value()); // Raw time in HHMMSSCC format (u32)
    Serial.println(gps.time.hour()); // Hour (0-23) (u8)
    Serial.println(gps.time.minute()); // Minute (0-59) (u8)
    Serial.println(gps.time.second()); // Second (0-59) (u8)
    Serial.println(gps.time.centisecond()); // 100ths of a second (0-99) (u8)
    Serial.println(gps.speed.value()); // Raw speed in 100ths of a knot (i32)
    Serial.println(gps.speed.knots()); // Speed in knots (double)
    Serial.println(gps.speed.mph()); // Speed in miles per hour (double)
    Serial.println(gps.speed.mps()); // Speed in meters per second (double)
    Serial.println(gps.speed.kmph()); // Speed in kilometers per hour (double)
    Serial.println(gps.course.value()); // Raw course in 100ths of a degree (i32)
    Serial.println(gps.course.deg()); // Course in degrees (double)
    -Serial.println(gps.altitude.value()); // Raw altitude in centimeters (i32)
    Serial.println(gps.altitude.meters()); // Altitude in meters (double)
    Serial.println(gps.altitude.miles()); // Altitude in miles (double)
    Serial.println(gps.altitude.kilometers()); // Altitude in kilometers (double)
    Serial.println(gps.altitude.feet()); // Altitude in feet (double)
  */
  Serial.print("Number of satellites in use   ");  Serial.println(gps.satellites.value()); // Number of satellites in use (u32)
  /*
    Serial.println(gps.hdop.value()); // Horizontal Dim. of Precision (100ths-i32)

  */



  /* gps.f_get_position(&flat, &flon, &age); //
    Serial.print(flon,6); //
    desimalt
    Serial.print(", "); //
    Serial.print(flat,6); //
    grader desimalt
    Serial.print(", "); //
    Serial.print(gps.f_altitude(), 2); //

  */
  Serial.println();
  
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis(); 
  do // 
  {
    while (mySerial.available())gps.encode(mySerial.read());
  }
  while (millis() - start < ms);
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
  char inData[20];
  char inChar = -1;
  byte index = 0;
  if (Serial2.available()){
    while (index<19) {
      inChar = Serial2.read();
      inData[index] = inChar;
      index++;
      //Serial.print(index);
    } 
    inData[index] = '\0';
    Serial.println(inData);
    for(int i=0;i<3;i++){
      int num = (inData[i*3]-'0')*100+(inData[i*3+1]-'0')*10+(inData[i*3+2]-'0')*1;
      Serial.print("Orient ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(num);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  //String str((char*)inData);
  //  Serial.println(str);
  //delay(100);

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
  //gps_init();

  radio_init();
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
  radio_update();
  //gps_update();
  smartdelay(1000);
  //delay(500);

}



