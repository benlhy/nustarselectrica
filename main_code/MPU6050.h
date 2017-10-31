#include<Wire.h>
class MPU6050 {
private:
    const double DEGCONV = 2000 / 32786.0;
    const int ADDRESS = 0x68;
    double gyro_pos[3];
    double frequency;
public:
    MPU6050(int frq);
    void init();
    void update();
    void update(double rad_vel[]);
    void pos(double pos[]);
};

