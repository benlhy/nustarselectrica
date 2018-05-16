#include "storage.h"


namespace nustars {
    Storage::Storage(String x) {
        isInitialized = false;
        fileName = new char[x.length()];
        for (int i = 0; i < x.length(); i++) {
            fileName[i] = x[i];
        }
        if (!SD.begin(BUILTIN_SDCARD)) {
            Serial.println("Card failed, or not present");
            return;
        }
        isInitialized = true;
    }

    void Storage::write(char* msg) {
        File dataFile = SD.open(fileName, FILE_WRITE);
        if (dataFile) {
            dataFile.println(msg);
            dataFile.close();
        }
    }
}