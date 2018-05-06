#include "PID.h"

namespace nustars {
    PID::PID() {
        desiredX = 0;
        modX = 0;
        previousError = 0;
        previousX = 0;
        accumulatedError = 0;
    }

    /**
     * Updates PID value information and sends appropriate signals to the motor.
     * @param sensor_x The current value of the X rotation in degrees
     */
    void PID::tick(int sensor_x) {
        int dx = previousX - sensor_x;
        if (dx >= 180) modX += 360;
        else if (dx <= -180) modX -= 360;
        previousX = sensor_x;
        currentError = sensor_x + modX - desiredX;

        accumulatedError += currentError;
        if (accumulatedError > ERROR_CAP) { //limit integrating effects
            accumulatedError = ERROR_CAP;
        } else if (accumulatedError < -ERROR_CAP) {
            accumulatedError = -ERROR_CAP;
        }
        int derivError = previousError - currentError;

        //PID
        double pidFactor = P * currentError + D * derivError + I * accumulatedError;

        double analogOut;
        if (pidFactor > 255) {
            analogOut = 255;
        } else if (pidFactor < -255) {
            analogOut = -255;
        } else {
            analogOut = pidFactor;
        }
        Serial.println(analogOut);
        //Serial.printf("Aout: %3d; cE: %d; SeX: %d\n", analogOut, currentError, sensor_x + modX);
        if (currentError < ZERO_TOLERANCE && currentError > ZERO_TOLERANCE) {
            digitalWrite(A8, 0);
            digitalWrite(A9, 0);
            analogWrite(A7, 0); //Maybe change this back to an actual value
            accumulatedError = 0;
        } else if (pidFactor > 0) {
            digitalWrite(A8, 1);
            digitalWrite(A9, 0);
            analogWrite(A7, analogOut);
        } else {
            digitalWrite(A8, 0);
            digitalWrite(A9, 1);
            analogWrite(A7, -analogOut);
        }
        previousError = currentError;
    }
}