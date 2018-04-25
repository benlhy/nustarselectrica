#ifndef _DEF_BNO
#define _DEF_BNO
#include <Adafruit_BNO055.h>
#endif

#ifndef  _DEF_BMP
#define _DEF_BMP
#include <Adafruit_BMP280.h>
#endif

#ifndef _DEF_ARDUINO
#define _DEF_ARDUINO
#include <Arduino> //TODO: Make sure this is correct
#endif

namespace nustars {
    class Sensor {
    public:
        void tick(); //Perform routine update operations (get values from the sensors etc
        const int X_AXIS = 0;
        const int Y_AXIS = 1;
        const int Z_AXIS = 2;
    };

    /**
     * Adafruit BNO, the accelerometer
     */
    class Accelerometer: Sensor {
    private:
        Adafruit_BNO55 bno;
        int lastX;
        int orientation[];
        int baseAlt;
    public:
        Accelerometer();
        int getOrientation(int axis);
        int getAcceleration(int axis);
    };

    /**
     * Adafruit BME, the altimeter
     */
    class Altimeter: Sensor {
    private:
        Adafruit_BMP280 bme;
        int temp, pressure, alt, baseAlt;
        void setBaseAlt();
    public:
        Altimeter();
        int getTemp();
        int getPressure();
        int getAltitude();
    };
}