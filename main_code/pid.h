#include "Arduino.h"

namespace nustars {
    class PID {
    private:
        //these are documented in PID.cpp
        int desiredX, previousX, modX, modDesire, previousError, currentError, accumulatedError;
        //PID constants
        const double P = 1;
        const double I = 0;
        const double D = 0;
        const int ZERO_TOLERANCE = 5; //+ or - from desiredX to shut off the motor (degrees)
        const int ERROR_CAP = 255; //limiting accumulated error to this value
    public:
        PID(); //TODO: Add pin options
        void tick(int x); //do PID stuff
        void setDesiredX(int x);
        int getDesiredX() const;
    };
}
