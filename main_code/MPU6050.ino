#include "MPU6050.h"
MPU6050 gyro;
void setup() {
  // put your setup code here, to run once:
  gyro.init();
  Serial.begin(9600);
  Serial.println(micros());
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("Hello");
  gyro.update(micros());
  double pos[3];
  gyro.pos(pos);
  
  
  Serial.print("x: "); Serial.println(pos[0]);
  Serial.print("y: "); Serial.println(pos[1]);
  Serial.print("z: "); Serial.println(pos[2]);
  
  
  delay(100);
}
