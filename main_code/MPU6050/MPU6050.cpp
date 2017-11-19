/**
    MPU6050 interface for Arduino
 */

#include "MPU6050.h"
#include <Wire.h>
#include <Arduino.h>

/**
 * Start communication with the unit, set its sensitivities.
 * This must be called when the Arduino is reset.
 */
void MPU6050::init() {
    Wire.begin();
    Wire.beginTransmission(ADDRESS);
    Wire.write(0x1B);   // Gyro register
    Wire.write(24);     // set to 11000 (+-2000deg/s)
    Wire.endTransmission(false);
    Wire.beginTransmission(ADDRESS);
    Wire.write(0x1C);   // Acc register
    Wire.write(16);     // set to 10000 (+-8g)
    Wire.endTransmission(false);
    Wire.beginTransmission(ADDRESS);
    Wire.write(0x6B);   // Power register
    Wire.write(0);      // set to 0 (good morning!)
    Wire.endTransmission(true); //release i2c
}

/**
 * Pulls velocity data from the gyroscope and recalculates position.
 * This method must be called on a regular basis to ensure accurate position calculations.
 * @param rad_vel must be double[3], is modified to contain {x,y,z} velocity data.
 * Values are in deg/s and range from [-2000, 2000]
 * @param time micros() from the Arduino main loop
 */
void MPU6050::update(double rad_vel[], unsigned long time) {
    Wire.beginTransmission(ADDRESS);
    Wire.write(0x43);  // starting at register 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(ADDRESS,6,true);  // requesting 6 registers
    unsigned long dt = time - last_update;
    if (1000000.0 / dt < MAX_REFRESH) {
        for (int i = 0; i < 3; i++) {
            rad_vel[i] = Wire.read() << 8 | Wire.read(); //reads two registers at a time for each dimension
            rad_vel[i] *= DEGCONV;
            gyro_pos[i] += rad_vel[i] * dt / 1000000.0;
        }
        last_update = time;
    }
    Wire.endTransmission(true); //release i2c
}

/**
 * Identical to update(double[]), but does not require passing a velocity variable.
 * @param time micros() from the Arduino main loop
 */
void MPU6050::update(unsigned long time) {
    double y[3];
    MPU6050::update(y, time);
}

/**
 * Modifies parameter to position data.
 * @param pos double[3] modified to {x,y,z} position data. Units are degrees relative to origin and are unbounded.
 */
void MPU6050::pos(double pos[]) {
    for (int i = 0; i < 3; i++) {
        pos[i] = gyro_pos[i];
    }
}
