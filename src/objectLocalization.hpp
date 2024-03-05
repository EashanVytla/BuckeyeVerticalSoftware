#ifndef OBJECTLOCALIZATION_H
#define OBJECTLOCALIZATION_H

//Includes
#ifndef OPENCV_CORE_MAT_HPP
#include <opencv2/opencv.hpp>
#endif

class objectLocalization
{
public:
    //object localization function
    //should take in target frame and output distance from camera to target
    std::vector<double> objectDistance(cv::Mat& targetFrame);

    //function to calculate target GPS coordinates
    std::vector<double> calcTargetCoordinates(std::vector<double> dist);
};  

#endif //OBJECTLOCALIZATION_H
