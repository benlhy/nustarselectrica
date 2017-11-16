#include<MPU6050.h>
int i = 0;
MPU6050 gyro(100);
void setup() {
  // put your setup code here, to run once:
  gyro.init();
  Serial.begin(9600);
  Serial.println("I'm alive!");
}

void loop() {
  // put your main code here, to run repeatedly:
  double x[3];
  gyro.update();
  gyro.pos(x);
  i++;
  if (i % 10 == 0) {
    Serial.print("X: "); Serial.println(x[0]);
    Serial.print("Y: "); Serial.println(x[1]);
    Serial.print("Z: "); Serial.println(x[2]);
    Serial.println();
  }
  delay(100);
}
