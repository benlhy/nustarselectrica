//  My First Rocket
#include 'display_header.h' 

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
  radio.send("Establishing comms");
  while(motor_init()){
    dis(1,0,0,0,0);
  }
  while(altitude_init()){
    dis(1,1,0,0,0);
  }
  while(gps_init()){
    dis(1,1,1,0,0);
  }
  while(accelometer_init()){
    dis(1,1,1,1,0);
  }
  dis(1,1,1,1,1);
  radio_send("Systems online");
}

// This function displays the status of the rocket
void dis(int radio, int motor,int alt, int gps, int acc){
  
}

