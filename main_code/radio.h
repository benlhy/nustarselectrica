#include "HardwareSerial.h"

namespace nustars {
    class Xbee {
    public:
        Xbee();
        HardwareSerial Serial2;
        void send() const;
        void receive(); //TODO: implement correct
    };
}
