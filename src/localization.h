#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>


using namespace cv;
using namespace std;

Mat projectPixel(double zConst, double u, double v,
                 cv::Mat &inverseCameraMatrix,
                 cv::Mat &inverseRotationMatrix,
                 cv::Mat &translationVector);


Mat alignedProject(double zConst, double u, double v, double originX, double originY,
                   cv::Mat &inverseCameraMatrix,
                   cv::Mat &inverseRotationMatrix,
                   cv::Mat &translationVector);

std::pair<double,double> offset_to_latlon(double lat,double lon,double heading,double offset_x,double offset_y);