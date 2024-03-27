#include "detect.h"
#include "iostream"

using namespace std;

enum class State{
    SCAN,
    DROP
}

State drone_state = State::SCAN; 

int main(){
    Detect detector();

    detector.model_on("C:/Data/Buckeye Vertical/Test Video/4xZoomCrop.mp4");

    switch(drone_state){
        case State::SCAN:
        {
            while(true){
                if (detect.getDetectedState())
                {
                    if (detectedSet.count(detect.getDetectedClassIdx()) == 0)
                    {
                        // only for testing
                        state = State::DROP;
                        myfile << "Changed state to DROP" << endl;

                        // reset flag for detecting object

                        sleep_for(400ms);
                        break;
                    }
                }
            }
        }
        case State::DROP:
        {
            cout << "DROPPING..." << endl;
            detector.model_off();
            return 1;
        }
    }

    detector.model_off();
}