#include "sensors.h"


namespace nustars {

    /**********ACCELEROMETER
     ***********************
     **********************/

    Accelerometer::Accelerometer() {
        bno = Adafruit_BNO055(55); //I2C address, probably.
        lastX = 0;
        orientation = new int[3];
        if (!bno.begin()) {
            Serial.print(
                    "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"); //TODO: Learn to throw an exception
            while (1);
        }
        bno.setExtCrystalUse(true);
    }

    /**
     * Update the orientation information
     */
    void Accelerometer::tick() {
        float collect[3];

        // The data will be very noisy, so we have to apply a moving average
        const int NUM_SAMPLES = 5;
        for (int i = 0; i < NUM_SAMPLES; i++) {
            sensors_event_t event;
            bno.getEvent(&event);
            collect[0] += event.orientation.x;
            collect[1] += event.orientation.y;
            collect[2] += event.orientation.z;
        }
        orientation[0] = collectX / NUM_SAMPLES;
        orientation[1] = collectY / NUM_SAMPLES;
        orientation[2] = collectZ / NUM_SAMPLES;

        // Relative
        changeX = lastX - orientX;
        if (changeX > 180) {
            // crossover from 360 to 0
            modifierX = modifierX + 360;
        } else if (changeX < -180) {
            modifierX = modifierX - 360;
        }
        relativeX = orientX + modifierX; // now we use relative X to calculate
        lastX = orientX;
    }

    /**
     * Get the current orientation
     * @param axis From Sensor class, integer representing axis to retrieve
     * @return The value of the orientation on requested axis
     */
    int Accelerometer::getOrientation(int axis) {
        switch(axis) {
            case X_AXIS: return orientation[0];
            case Y_AXIS: return orientation[1];
            case Z_AXIS: return orientation[2];
            default: return -1; //TODO: Throw an exception
        }
    }

    /**
     * Get the current acceleration
     * @param axis From Sensor class, integer representing axis to retrieve
     * @return The value of the acceleration on requested axis
     */
    int Accelerometer::getAcceleration(int axis) {
        switch(axis) {
            case X_AXIS: return bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER).x;
            case Y_AXIS: return bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER).y;
            case Z_AXIS: return bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER).z;
            default: return -1; //TODO: Throw an exception
        }
    }


    /***********ALTIMETER
     ********************
     *******************/

    /**
     * Initialize the BME and perform starting tasks
     */
    Altimeter::Altimeter() {
        if (!bme.begin()) {
            Serial.println("Could not find a valid BMP280 sensor, check wiring!");
            while (1);
        }
        setBaseAlt();
    }

    /**
     * Take several samples of the current altitude and record the average
     */
    void Altimeter::setBaseAlt() {
        const int NUM_BASE_SAMPLES = 10;
        //Set altitude at ground
        baseAlt = 0;
        for (int i = 0; i < NUM_BASE_SAMPLES; i++) {
            baseAlt += bme.readAltitude(1000);
        }
        baseAlt = baseAlt / NUM_BASE_SAMPLES; //average readings
    }

    /**
     * Update the current altitude, pressure, and temperature
     */
    void Altimeter::tick() {
        temp = bme.readTemperature();
        pressure = bme.readPressure();
        alt = bme.readAltitude(1000) - baseAlt;
    }

    //Altimeter getters--

    //get temperature
    int Altimeter::getTemp() {
        return temp;
    }

    //get altitude
    int Altimeter::getAltitude() {
        return alt;
    }

    //get pressure
    int Altimeter::getPressure() {
        return pressure;
    }
} //END NAMESPACE

