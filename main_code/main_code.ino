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

Adafruit_BMP280 bme; // I2C

int curr_alt = 0;

////////////////////////// MOTOR TEAM /////////////////////////
void motor_init() {
  pinMode(A9, INPUT); //motor direction input A
  pinMode(A8, INPUT); //motor direction input B
  pinMode(A7, PWM); //controls motor speed
}
// update the pwm signal based on input from sensors
void motor_calc (){
  int prevEI;
  float pE, iE, dE;
  float dAvg[5] = {prevE, prevE, prevE, prevE, prevE};
  int dAvgPoint = 0;
  int waitTimeConstant = waitTime;
  float initialVelocityThreshold = 180;
  enum flightPlan_t { RESET, PID } plan = RESET;
  int calculatePID() {
      float u;
      int dt = currentData.time - prevData.time;
      float error = 0;
      error = calculateError();
    
      // Scales pE
      pE = error * kp;
    
      // calculates Moving average derivative
      dAvg[dAvgPoint] = (error - prevE) / dt;
      dAvgPoint++;
      dAvgPoint = dAvgPoint % 5;
      dE = 0;
      for (int i = 0; i < 5; i++) {
          dE += dAvg[i];
      }
      dE = dE / 5;

      // Sets prevE to be this eror
      prevE = error;

      // Sets prevEI to be running sum
      prevEI += error * dt;


      // Anti integrator
      if (prevEI > 60 / ki) { 
          prevEI = 60 / ki;
      }
      if (prevEI < -60 / ki) {
          prevEI = -60 / ki;
      }

      // scales iE
      iE = prevEI * ki;

      // scales dE
      dE = kd * dE;

      // Sum of everything is the motor output
      u = iE + dE + pE;

      return u;
  }

  // Takes in power and turns it into a directional output
  void outputMotor(int power) {
      // Positive power is direction Pin HIGH.
      if (power > 0) {
          digitalWrite(directionPin, HIGH);
          analogWrite(speedPin, abs(power));
      }
      else { // else its 0 or negative. Either way, direction is LOW
          digitalWrite(directionPin, LOW);
          analogWrite(speedPin, abs(power));
      }
  }

  // Calculates PID value and outputs to motor.
  void callThemAll(uint32_t startTime) {
     plan = PID;


          // u is the number to output to the Motor
          int u = 0;

          // if the plan is to PID the controller
          if (plan == PID) {
              // gets PID calculations
              u = calculatePID();
          }
              // Else if the plan is to Reset, we call the resetVelocity function
          else{
              // sets u to the velocity given by the resetVelocity function
              u = resetVelocity();
          }

          // limits u to +- 255, though it should be fine without this

          if ( u < -255) {
              u = -255;
          }
          // Outputs that to the motor

          if (u > 255) {
              u = 255;
          }
          outputMotor(u);
      }
  }

  // Generates a linear trajectory to reach the desired position.
  float calculateError(void) {
      // get time elasped, which is currentTime - launchTime - waitTime
      int timeElapsed = currentData.time - launchTimestamp - waitTimeConstant;
      if (timeElapsed < 0)
      {
          return 0;
      }
      // this is kinda like how far from the target we should be
      //So this returns anywhere from 0.8 of the original turnLeft to 0;
      float errorT = 0.8 * ogTurnLeft - (0.8 * ogTurnLeft * timeElapsed) / targetTime;

      // we dont want this to keep going forever
      if (((errorT < 0) && (ogTurnLeft > 0)) || ((errorT > 0) && (ogTurnLeft < 0))) {
          errorT = 0;
      }

      // we take turnLeft - errorT to get a line going from 20% of turnLeft to 100% of turnLeft in targetTime
      errorT = turnLeft - errorT;

      return errorT;
  }

  void resetController(void) {
      prevEI=0;

      ogTurnLeft = abs(ogTurnLeft);
      turnLeft = ogTurnLeft;
      prevE = 0.2*turnLeft;
      powerG=0;
      isLaunchGyroSet = 0;
      pE=0;
      iE=0;
      dE=0;
      dAvgPoint = 0;
      initalVelocityThreshold = 180;
      plan = RESET;
  }
  }

void motor_update() {
int calculatePID();

//outputs the PID output to the motor, sanitizing as necessary
void outputMotor(int power);

//Calls the other functions in sequence to get an output to the motor
void callThemAll(uint32_t timestamp);

//generates a linear trajectory error path for us to follow
float calculateError();

//resets controller
void resetController();
}

///////////////////////////////////////////////////////////////

///////////////////////// RADIO TEAM //////////////////////////
void radio_init() {
  Serial2.begin(9600); // USE TX/RX 2 of Teensy 3.5
}

void update_radio() {
  
}
///////////////////////////////////////////////////////////////

//////////////////////////CAMERA CODE//////////////////////////
int trig = 0;
int led = 1;

void cameraSetup() {
  // initialize digital pins as output
  pinMode(led, OUTPUT);
  pinMode(trig, OUTPUT);

  digitalWrite(led,HIGH);
  digitalWrite(trig, HIGH);

}

void cameraLoop() {
  // set a single loop for the camera to run
  digitalWrite(trig, LOW);
  digitalWrite(led, LOW);

  delay(150);

  digitalWrite(trig, HIGH);
  digitalWrite(led, HIGH);

  delay(500);//how long before it stops recording video
}

/////////////////////////////////////////////////////////////
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



