
// C++ program for the above approach
#include <iostream>

#include <opencv2/opencv.hpp>
#include "localization.h"

using namespace cv;
using namespace std;

// Driver code
int main(int argc, char **argv)
{

    int rows = 2;
    int cols = 1;

    cout << "worked here" << endl;

    Mat testmat(rows, cols, CV_32FC1);

    testmat.at<float>(0, 0) = 1;
    testmat.at<float>(1, 0) = 2;

    // cout << (2 * testmat) << endl;

    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;
    cv::Mat rotationMatrix;
    cv::Mat translationVector;

    cv::FileStorage fi("t_intrinsics.xml", cv::FileStorage::READ);
    cv::FileStorage fe("t_extrinsics.xml", cv::FileStorage::READ);

    fi["cameraMatrix"] >> cameraMatrix;
    fi["distortion"] >> distortionCoeffs;

    fe["rotationMatrix"] >> rotationMatrix;
    fe["translationVector"] >> translationVector;

    fi.release();
    fe.release();

    // cout << cameraMatrix << endl;

    // cout << endl;
    // cout << "GOT TO HERE" << endl;
    
    Mat inverseCameraMatrix = cameraMatrix.inv();
    Mat inverseRotationMatrix = rotationMatrix.inv();

    // cout << endl;
    // cout << "PAST INVERSE" << endl;

    // cout << "CAMERA MATRIXC" << endl << cameraMatrix << endl;
    // cout << "INV MATRIX" << endl << inverseCameraMatrix << endl;


    // double u = 1112;
    // double v = 119;

    double u = 967;
    double v = 624;


    const double INCHES_PER_METER = 39.3701;
    const double METERS_PER_INCH = 1 / INCHES_PER_METER;
    const double INCHES_PER_CHECKER = 0.875;
    const double METERS_PER_CHECKER = INCHES_PER_CHECKER * METERS_PER_INCH;
    const double CHECKERS_PER_INCH = 1 / INCHES_PER_CHECKER;
    const double INCHES_OFFSET = 58.125;



// (1095, 652)
    double originX = 640;
    double originY = 360;

    Mat alignedPoint = alignedProject(100.0, u, v, originX, originY, inverseCameraMatrix, inverseRotationMatrix, translationVector);

    cout << alignedPoint << endl;

    double deltaX = alignedPoint.at<double>(0) * METERS_PER_CHECKER;
    double deltaY = alignedPoint.at<double>(1) * METERS_PER_CHECKER;

    cout << deltaX << std::endl;
    cout << deltaY << std::endl;

    return 0;
}