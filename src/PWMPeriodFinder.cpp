#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

using namespace std;

const string GPIO_PATH = "/sys/class/gpio/";

// Function to export a GPIO pin
void exportGPIO(int gpioPin) {
    ofstream exportFile(GPIO_PATH + "export");
    if (exportFile.is_open()) {
        exportFile << gpioPin;
        exportFile.close();
    } else {
        cerr << "Unable to export GPIO " << gpioPin << endl;
    }
}

// Function to set the direction of a GPIO pin (in or out)
void setGPIODirection(int gpioPin, const string& direction) {
    ofstream directionFile(GPIO_PATH + "gpio" + to_string(gpioPin) + "/direction");
    if (directionFile.is_open()) {
        directionFile << direction;
        directionFile.close();
    } else {
        cerr << "Unable to set direction for GPIO " << gpioPin << endl;
    }
}


// Function to read the value of a GPIO pin
int readGPIOValue(int gpioPin) {
    ifstream valueFile(GPIO_PATH + "gpio" + to_string(gpioPin) + "/value");
    if (valueFile.is_open()) {
        int value;
        valueFile >> value;
        valueFile.close();
        return value;
    } else {
        cerr << "Unable to read value from GPIO " << gpioPin << endl;
        return -1;
    }
}

int main() {
    // Specify the GPIO pin number
    int gpioPin = 168; // Change this to your GPIO pin number

    // Export GPIO pin
    exportGPIO(gpioPin);

    // Set GPIO direction (input)
    setGPIODirection(gpioPin, "in");

    int value = 0;
    int prevVal = 0;

    chrono::steady_clock::time_point lastSwitch = chrono::steady_clock::now();

    while(true){
        // Read GPIO value
        value = readGPIOValue(gpioPin);

        if(value != prevVal){
            chrono::steady_clock::time_point currentTime = chrono::steady_clock::now();
            int elapsedTime = chrono::duration_cast<std::chrono::microseconds>(currentTime - lastSwitch).count();

            if(prevVal > value && elapsedTime > 1200){
                cout << "WE MADE IT!!!!" << endl;
            }

            cout << "Value: " << value << " Elapsed TIme: " << elapsedTime << '\n';
            lastSwitch = std::chrono::steady_clock::now();
        }

        prevVal = value;
    }

    return 0;
}
