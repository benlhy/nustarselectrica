


namespace nustars {
    class Accelerometer: Sensor {
    private:
        Adafruit_BNO55 bno;
        int lastX;
        int orientation;
    public:
        int baseAlt = 0;
        Accelerometer() {
            bno = Adafruit_BNO055(55); //I2C address, probably.
            lastX = 0;
            orientation = new int[3];
            if (!bno.begin())
            {
                Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"); //TODO: Learn to throw an exception
                while (1);
            }
            bno.setExtCrystalUse(true);
        }
        void tick() {
            float collectX = 0;
            float collectY = 0;
            float collectZ = 0;
            int orientX, orientY, orientZ;
            // The data will be very noisy, so we have to apply a moving average
            for (int i = 0; i < 5; i++) {
                sensors_event_t event;
                bno.getEvent(&event);
                collectX = event.orientation.x + collectX;
                collectY = event.orientation.y + collectY;
                collectZ = event.orientation.z + collectZ;
            }
            orientX = collectX / 5;
            orientY = collectY / 5;
            orientZ = collectZ / 5;

            // Relative
            changeX = lastX - orientX;
            if (changeX>180){
                // crossover from 360 to 0
                modifierX = modifierX + 360;
            }
            else if (changeX<-180){
                modifierX = modifierX - 360;
            }
            relativeX = orientX + modifierX; // now we use relative X to calculate
            lastX = orientX;

            //TODO: BME CODE WHY
            temp = bme.readTemperature();
            pressure = bme.readPressure();
            alt = bme.readAltitude(1000)-baseAlt;


        }
    };
    class Altimeter: Sensor {
    private:
        Adafruit_BMP280 bme;
    public:
        Altimeter() {
            if (!bme.begin()) {
                Serial.println("Could not find a valid BMP280 sensor, check wiring!");
                while (1);
            }
            //Set altitude at ground
            for (int i = 0; i < 10; i++) {
                baseAlt = baseAlt + bme.readAltitude(1000);
            }
            baseAlt = baseAlt/10;
        }
    };
}

