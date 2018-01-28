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
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BMP280 bme; // I2C

Adafruit_BNO055 bno = Adafruit_BNO055(55);

int curr_alt = 0;
int prevE = 0;
int prevEI = 0;
int prevED = 0;
int currE = 0;
int controlu = 0;


float orientX = 0;
float orientY = 0;
float orientZ = 0;

float desiredX = 0;
float desiredY = 0;
float desiredZ = 0;

int Kp=0,Kd=0,Ki=0;
/////////////////////////// SENSOR TEAM   /////////////////////
void sensor_init() {
    if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);
  
}

void sensor_update() {
  sensors_event_t event;
  bno.getEvent(&event);
  float collectX=0;
  float collectY=0;
  float collectZ=0;
  // The data will be very noisy, so we have to apply a moving average
  for (int i=0;i<10;i++){
    sensors_event_t event;
    bno.getEvent(&event);
    collectX=event.orientation.x + collectX;
    collectY=event.orientation.y + collectY;
    collectZ= event.orientation.z + collectZ;
    delay(5);
  }
  orientX = collectX/10;
  orientY = collectY/10;
  orientZ = collectZ/10;
}
////////////////////////// MOTOR TEAM /////////////////////////
void motor_init() {
  pinMode(A9, OUTPUT); //motor direction input A
  pinMode(A8, OUTPUT); //motor direction input B
  pinMode(A7, 0); //controls motor speed
}
// update the pwm signal based on input from sensors
void motor_update (){

  currE = desiredX - orientX;
  prevED = prevE - currE;
  prevEI = prevEI + currE;
  // anti integrator
  if (prevEI>60){
    prevEI = 60;
  }
  if (prevEI<-60){
    prevEI = -60;
  }
  controlu = Kp*currE+Kd*prevED+Ki*prevE;
  prevE = currE;
  if (controlu>255){
    digitalWrite(A8,1);
    digitalWrite(A9,0);
    analogWrite(A7,255);
  }
  else if ((controlu<0)&&(controlu>-255)){
    digitalWrite(A8,0);
    digitalWrite(A9,1);
    analogWrite(A7,-controlu);
  }
  else if (controlu<-255){
    digitalWrite(A8,0);
    digitalWrite(A9,1);
    analogWrite(A7,255);
  }
  else if (controlu>0&&controlu<255){
    digitalWrite(A8,1);
    digitalWrite(A9,0);
    analogWrite(A7,controlu);
  }
  else if(controlu==0){
    digitalWrite(A8,0);
    digitalWrite(A9,0);
    analogWrite(A7,controlu);
  }
}



///////////////////////////////////////////////////////////////

///////////////////////// RADIO TEAM //////////////////////////
void radio_init() {
  Serial2.begin(9600); // USE TX/RX 2 of Teensy 3.5
  for (int i =0; i<10; i++){
    Serial2.write("Radio check");
  }
}

void radio_update() {
  char reply[20];
  Serial2.write("Hello back 2 you.");
  
}
///////////////////////////////////////////////////////////////

//////////////////////////CAMERA CODE//////////////////////////
int trig = 0;
int led = 1;

void camera_setup() {
  // initialize digital pins as output
  pinMode(led, OUTPUT);
  pinMode(trig, OUTPUT);

  digitalWrite(led,HIGH);
  digitalWrite(trig, HIGH);

}
///////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////
void setup() {
  
  Serial.begin(9600);
  delay(1000);
  Serial.println("Hello");

  radio_init();
  camera_setup();
  sensor_init();
  motor_init();
  
}

void loop() {


  // Here we update the sensors
    sensor_update();// delay for 50ms
  // Based on the sensors, pass sensor data to motors
    motor_update();
  // update groundstation
    radio_update(); 
 
}



