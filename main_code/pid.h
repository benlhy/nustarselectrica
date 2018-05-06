#include "Arduino.h";

namespace nustars {
    class PID {
    private:
        int desiredX, previousX, modX, previousError, currentError, accumulatedError;
        //accumulatedError: sloppy integration of the error
        const double P = 2;
        const double I = .2;
        const double D = 2; //TODO: Implement
        const int ZERO_TOLERANCE = 5; //+ or - from desiredX to shut off the motor
        const int ERROR_CAP = 255; //limiting accumulated error
    public:
        PID(); //TODO: Add pin options
        void tick(int x); //do PID stuff
        void setDesiredX(int x);
    };
}
