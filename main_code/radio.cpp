#include "radio.h"


namespace nustars {
    Xbee::Xbee() {
        Serial2.begin(9600);
        HardwareSerial Serial2(2);
    }
    void Xbee::send() const {
        Serial2.write("HI");
    }
}