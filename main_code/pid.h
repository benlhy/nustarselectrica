#include "Arduino.h";

namespace nustars {
    class PID {
    private:
        //these are documented in PID.cpp
        int desiredX, previousX, modX, previousError, currentError, accumulatedError;
        //PID constants
        const double P = 2;
        const double I = .2;
        const double D = 2;
        const int ZERO_TOLERANCE = 5; //+ or - from desiredX to shut off the motor (degrees)
        const int ERROR_CAP = 255; //limiting accumulated error to this value
    public:
        PID(); //TODO: Add pin options
        void tick(int x); //do PID stuff
        void setDesiredX(int x);
    };
}
