#include "SD.h"

namespace nustars {
    class Storage {
    public:
        Storage(String);
        void write(String);
        int* read(int& lng);

    private:
        File sdFile;
        String fileName;
    };
}
