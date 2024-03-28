#include "detect.h"

#include <set>
#include <string>



#include "iostream"

using namespace std;
using std::this_thread::sleep_for;
using std::chrono::seconds;

enum class State{
    SCAN,
    DROP
};

State drone_state = State::SCAN; 
set<int> detectedSet;
set<int> targetSet;


int getIdx(Detect& detector, string name) {

    for (int i = 0; i < detector.getClassNames().size(); i++)
        if (name == detector.getClassNames().at(i))
            return i;

    return -1;

}


int main(){

    Detect detector;

    cout << "idx for Blue_Rectangle " << getIdx(detector, "Blue_Rectangle") << endl;

    targetSet.insert(getIdx(detector, "Blue_Rectangle"));
    targetSet.insert(getIdx(detector, "Black_Star"));
    targetSet.insert(getIdx(detector, "White_Semicircle"));
    targetSet.insert(getIdx(detector, "Blue_Cross"));
    targetSet.insert(getIdx(detector, "Red_Circle"));

    cout << "Starting model" << endl;
    detector.model_on("/media/buckeyevertical/A870C69770C66B9E/Test-Data/Videos-20240227T195339Z-001/Videos/4xZoomCrop.mp4");

    int counter = 0;

    int currentDropTarget = 0;

    while(true){
        switch(drone_state){
            case State::SCAN:
            {
                if (detector.getDetectedClassIdx() == -1)
                    continue;

                if (targetSet.count(detector.getDetectedClassIdx()) > detectedSet.count(detector.getDetectedClassIdx()))
                {
                    // only for testing
                    drone_state = State::DROP;

                    currentDropTarget = detector.getDetectedClassIdx();

                    cout << "Changed state to DROP" << endl;
                    // break;

                    // counter++;
                    // reset flag for detecting object

                    
                }

                // sleep_for(1ms);
                
            }
            break;

            case State::DROP:
            {
                // counter = 0;
                
                cout << endl << endl;
                cout << "DROPPING FOR..." << detector.getClassNames().at(currentDropTarget) << endl;
                cout << endl << endl;

                detectedSet.insert(currentDropTarget);
                // detector.setDetectedState(false);

                drone_state = State::SCAN;
                // detector.model_off();
                // return 1;
            }
            break;
        }
    }
}