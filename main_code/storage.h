#include "SD.h"

namespace nustars {
    class Storage {
    public:
        Storage(String file);
        void write(String msg);
        int* read(int& lng);

    private:
        File sdFile;
        String fileName;
    };
}
