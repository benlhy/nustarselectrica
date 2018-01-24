//  My First Rocket
//#include "display_header.h" 
//#include "radio_functions.h"
//#include "MPU6050/MPU6050.h"

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

Adafruit_BMP280 bme; // I2C
sensors_event_t GYRO;


int curr_alt = 0;

////////////////////////// MOTOR TEAM /////////////////////////
void motor_init() {
  // setup pinmodes
}
// update the pwm signal based on input from sensors
void motor_calc (){
  // do calculations here
}

void motor_update() {
  // update signal to motors here
}
///////////////////////////////////////////////////////////////

///////////////////////// SENSOR TEAM /////////////////////////
void sensor_init() {
  // setup pinmodes and check
  if (!bme.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  if (!bno.begin()) {
    Serial.println("Could not find a valid BNO055.");
    while (1);
  }
}

void update_sensors() {
  // BME
  curr_alt = bme.readAltitude(1013.25);
  Serial.print("Approx altitude = ");
  Serial.print(curr_alt); // this should be adjusted to your local forcase
  Serial.println(" m");

  // BNO
  bno.getEvent(&GYRO);
  // GYRO.orientation.(x/y/z)
  Serial.print("X axis orientation: ");
  Serial.println(GYRO.orientation.x);

}
///////////////////////////////////////////////////////////////

///////////////////////// RADIO TEAM //////////////////////////
void radio_init() {
  Serial2.begin(9600); // USE TX/RX 2 of Teensy 3.5
}

void update_radio() {
  
}
///////////////////////////////////////////////////////////////

void setup() {
  
  Serial.begin(9600);
  
  delay(1000);
  Serial.println("Hello");

  radio_init();

  sensor_init();

  
  motor_init();
  // some code to start
  //init_all();
  //screen_huh(); // check if display is on or off
  //spin_parameters_huh(); // ask ground station if we want to set new spin parameters
  //motor_test_huh(); // do we want a motor test?
}

void loop() {
  // keep on spinning
    sensor_update();
    motor_calc();
    motor_update();


}



