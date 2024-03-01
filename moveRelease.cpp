//This code should have the drone process a frame passed into it and find targets coordinates. Then move to target and drop package

//Includes:
#include "opencv2/opencv.hpp"

//maken object localization script into a method.
//should take in target frame and output distance from camera to target
double objectLocalization(cv::Mat& targetFrame){

}

//this method should take the returned distance from objectLocalization method and calculate GPS coordinates of target
double calcTargetCoordinates(double dist){

}

//this method should take in the object class and move the specific servo relating to the target. 
void dropPackage(int targetClass){

}


int main(){
    int objClass = 0;
    int height = 80;
    cv::Mat frame;

    //In main we should do the following tasks:

    //FYI might be array instead of double for distance
    double distance = objectLocalization(frame);
    double targetCoordinates = calcTargetCoordinates(distance)

    //Move drone to these coordinates
    //Once above target dropn to "height"

    dropPackage(objClass);

    return 0;
}

