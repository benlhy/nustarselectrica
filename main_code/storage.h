#include "SD.h"
#include <SPI.h>

namespace nustars {
    class Storage {
    public:
        Storage(String file);
        void write(char* msg);
        int* read(int& lng);

    private:
        bool isInitialized;
        char* fileName;
    };
}
