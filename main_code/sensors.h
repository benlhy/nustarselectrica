#ifndef _DEF_BNO
#define _DEF_BNO
#include <Adafruit_BNO055.h>
#endif

#ifndef  _DEF_BMP
#define _DEF_BMP
#include <Adafruit_BMP280.h>
#endif

namespace nustars {
    static const int X_AXIS = 0;
    static const int Y_AXIS = 1;
    static const int Z_AXIS = 2;

    class Sensor {
        virtual void tick();
    };
    /**
     * Adafruit BNO, the accelerometer
     */
    class Accelerometer: public Sensor {
    private:
        Adafruit_BNO055 bno;
        int lastX;
        int modifierX;
        int* orientation;
        int baseAlt;
    public:
        Accelerometer();
        void tick();
        int getOrientation(int axis);
        int getAcceleration(int axis);
    };

    /**
     * Adafruit BME, the altimeter
     */
    class Altimeter: public Sensor {
    private:
        Adafruit_BMP280 bme;
        int temp, pressure, alt, baseAlt;
        void setBaseAlt();
    public:
        Altimeter();
        void tick();
        int getTemp();
        int getPressure();
        int getAltitude();
    };
}
