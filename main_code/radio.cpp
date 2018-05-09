#include "radio.h"
#include "Arduino.h"

namespace nustars {
    bool Radio::checkLife() {
        String str;
        if (Serial2.available()) {
            str = Serial2.readString();
            lastContact = millis();
            return true;
        } else {
            return (millis() - lastContact < TIMEOUT);
        }
    }

    void Radio::tick() {
        alive = checkLife();
    }

    bool Radio::isAlive() const {
        return alive;
    }
}
