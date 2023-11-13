#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

using namespace std;

class PWMUtils{
    public:
        bool waitTillChannel(int pin, int threshold);

    private:
        void exportGPIO(int pin);
        void setGPIODirection(int pin, const string& direction);
        int readGPIOValue(int pin);
};
