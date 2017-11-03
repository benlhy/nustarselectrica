class MPU6050 {
private:
    const double DEGCONV = 2000 / 32786.0;
    const int ADDRESS = 0x68;
    const int MAX_REFRESH = 2000; //max refresh rate in Hz
    double gyro_pos[3];
    unsigned long last_update = 0;
public:
    void init();
    void update(unsigned long);
    void update(double rad_vel[], unsigned long);
    void pos(double pos[]);
};
