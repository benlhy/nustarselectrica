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
    };
    class Accelerometer: Sensor {

        Adafruit_BNO55 bno;
        int lastX;
        int orientation;
    };
    class Altimeter: Sensor {

    };
}