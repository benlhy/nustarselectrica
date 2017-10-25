//  My First Rocket
#include "display_header.h" 
#include "radio_functions.h"

void setup() {
  // some code to start
  init_all();
  //screen_huh(); // check if display is on or off
  //spin_parameters_huh(); // ask ground station if we want to set new spin parameters
  //motor_test_huh(); // do we want a motor test?
}

void loop() {
  // keep on spinning

}


// This function begins all functions
void init_all() {
  Serial.begin(9600);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
  display.display();
  display.clearDisplay();
  while(radio_init()){
    dis(0,0,0,0,0);
  }
  // Okay radio initialized, tell ground that we are alive.
  rf95.send((uint8_t *)"Hello ground. AMI beginning initialisation.", 44);
  while(motor_init()){
    dis(1,0,0,0,0);
  }
  rf95.send((uint8_t *)"Motor online.",14);
  while(altitude_init()){
    dis(1,1,0,0,0);
  }
  rf95.send((uint8_t *)"Altitude online. Current altitude is: ",39);
  while(gps_init()){
    dis(1,1,1,0,0);
  }
  rf95.send((uint8_t *)"GPS online. Current location is: ",34);
  while(accelometer_init()){
    dis(1,1,1,1,0);
  }
  rf95.send((uint8_t *)"Accelometer online. Current velocity is: ",42);
  dis(1,1,1,1,1);
  rf95.send((uint8_t *)"All systems online. AMI handling control to ground.",52);
}

// This function displays the status of the rocket
void dis(int radio, int motor,int alt, int gps, int acc){
  
}

// dummy functions
bool motor_init(){
  return true;
}

bool altitude_init(){
  return true;
}

bool gps_init(){
  return true;
}
bool accelometer_init(){
  return true;
}

