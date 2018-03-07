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
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define mySerial Serial1

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (1)

const int chipSelect = BUILTIN_SDCARD;
File dataFile;

Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true

Adafruit_BMP280 bme; // I2C

Adafruit_BNO055 bno = Adafruit_BNO055(55);

TinyGPSPlus gps; // define tinygps as gps
//Wire.setClock(400000);

int prevE = 0;
int prevEI = 0;
int prevED = 0;
int currE = 0;
int controlu = 0;


int orientX = 0;
int orientY = 0;
int orientZ = 0;
float collectX = 0;
float collectY = 0;
float collectZ = 0;

int temp = 0;
int pressure = 0;
int alt = 0;
int baseAlt = 0;

float desiredX = 0;
float desiredY = 0;
float desiredZ = 0;

int Kp = 0, Kd = 0, Ki = 0;

float currLat = 0, currLon = 0;
int heartbeat = 0;
uint32_t timer = millis();

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
  currLon = gps.location.lng();
  currLat = gps.location.lat();

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
  for (int i =0; i<10; i++){
    baseAlt = baseAlt + bme.readAltitude(1000);
  }
  baseAlt = baseAlt/10;
  
}


void sensor_update() {
  collectX = 0;
  collectY = 0;
  collectZ = 0;
  // The data will be very noisy, so we have to apply a moving average
  for (int i = 0; i < 5; i++) {
    sensors_event_t event;
    bno.getEvent(&event);
    collectX = event.orientation.x + collectX;
    collectY = event.orientation.y + collectY;
    collectZ = event.orientation.z + collectZ;
  }
  orientX = collectX / 5;
  orientY = collectY / 5;
  orientZ = collectZ / 5;
  temp = bme.readTemperature();
  pressure = bme.readPressure();
  alt = bme.readAltitude(1000)-baseAlt;
  /*
  Serial.print("Temperature = ");
  Serial.print(temp);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" Pa");

  Serial.print("Approx altitude = ");
  Serial.print(alt); // this should be adjusted to your local forcase
  Serial.println(" m");
  */
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
  // anti integrator, stops integrating once we saturate
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
    Serial.print("Maintaining position!");
  }
  
}



///////////////////////////////////////////////////////////////

///////////////////////// RADIO TEAM //////////////////////////
void sd_init()
{

  Serial.print("Initializing SD card...");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}

void radio_init() {
  Serial2.begin(9600); // USE TX/RX 2 of Teensy 3.5
  for (int i = 0; i < 10; i++) {
    Serial2.write("Radio check");
  }

}
// Example
// [confirm] [feedback] [orient] [termination]
//    BN         2      360180220     E
// [confirm] [command]
//    BN         K (read back all data from SD card)
void radio_update() {
  char inData[20];
  char inChar = -1;
  int index = 0;
  int feedback = 0;
  if (Serial2.available()){
    
    while (inChar != 'E') {
      // terminating character
      inChar = Serial2.read();
      inData[index] = inChar;
      index++;
    } 
    Serial.print(index);
    inData[index] = '\0';
    Serial.println(inData);
    if ((inData[0]=='B') && (inData[1]=='N')){
      feedback = (inData[2]-'0')*2;
      for(int i=0;i<((index-2)/3);i++){
        int num = (inData[i*3+2]-'0')*100+(inData[i*3+3]-'0')*10+(inData[i*3+4]-'0')*1;
        Serial.print("Orient ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(num);
        Serial.print(" ");
      }
      Serial.println();
      Serial2.write("Okay, rotate parameters updated. ");
    }
  }
  
  //String str((char*)inData);
  //  Serial.println(str);
  //delay(100);

  heartbeat = heartbeat + 1;

  char reply[20];
  Serial2.write("Ground Station: ");
  Serial.print("Sending val: ");
  sprintf(reply, "F:%d H:%d X:%d Y:%d Z:%d ",feedback, heartbeat, orientX, orientY, orientZ);
  Serial2.write(reply);
  Serial.print(reply);
  char reply2[20];
  sprintf(reply2, "A:%d P:%d T:%d ", alt, pressure, temp);
  Serial2.write(reply2);
  Serial.print(reply2);
  char reply3[30];
  sprintf(reply3, "LA:%f LO:%f", currLat, currLon);
  Serial2.write(reply3);
  Serial.println(reply3);
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print(reply);
    dataFile.print(reply2);
    dataFile.println(reply3);
    dataFile.close();
  }

// Read off SD to XBee

  if ((inData[2]=='S') && (inData[3]=='D')) {
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
      Serial.println("datalog.txt:");

      while (dataFile.available()) {
        xbee.write(dataFile.read());
      }
      
      dataFile.close();
    } else {
      Serial.println("error opening datalog.txt");
    }
  }
  
}

///////////////////////////////////////////////////////////////

//////////////////////////CAMERA CODE//////////////////////////
int cameraCounter = 0;
int trig = 2;
void camera_setup(){
  
  pinMode(trig, OUTPUT);
  digitalWrite(trig, HIGH);
  delay(50);
  digitalWrite(trig,LOW);
  //delay(10000);
  //digitalWrite(trig, LOW);
  
}

void camera_update(){
  if (cameraCounter<100){
    cameraCounter++;

  }
  else {
    cameraCounter=0;
    digitalWrite(trig,HIGH);
    delay(10);
    digitalWrite(trig,LOW);
    delay(100);
    digitalWrite(trig,HIGH);
    delay(50);
    digitalWrite(trig,LOW);
  }
}
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
//long lasttime = millis();

/////////////////////////////////////////////////////////////
void setup() {
  delay(3000);

  Serial.begin(115200);
  delay(1000);
  Serial.println("NUSTARS Fundation Electrica");
  Serial.println("Confirmation code: BN");
  Serial.println("Termination code: E");
  Serial.println("Send SD data code: SD");
  Serial.println("Currently accepting confirmation code, feedback value, orientation (0-360) positions at 1 sec intervals, termination code.");
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  gps_init();
  sd_init();
  dataFile = SD.open("datalog.txt", FILE_WRITE);

  radio_init();
  camera_setup();
  sensor_init();
  //motor_init();


}


long lasttime = millis();
long mytime = millis();
long diff = 0;
int toggle = 1;
void loop() {
  digitalWrite(13,~toggle);
  mytime = millis();
  diff = mytime - lasttime;
  lasttime = millis();
  Serial.print("Time per loop: ");
  Serial.println(diff);
  camera_update();


  // Here we update the sensors
  sensor_update();// delay for 1ms
  // Based on the sensors, pass sensor data to motors
  //motor_update();
  // update groundstation
  radio_update();
  gps_update();
  smartdelay(10);
  

}

