#pragma once
#ifndef DETECT_H
#define DETECT_H

#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <cmath>
#include <math.h>
#include <fstream>
#include <set>
#include <string>
#include "opencv2/opencv.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/param/param.h>

using namespace cv;
using namespace std;

class Detect
{
public:
    Detect();

    std::deque<cv::Mat> frame_buffer;

    std::mutex buffer_lock;
    std::mutex inference_lock;
    
    std::atomic<bool> is_running = false;

    // std::atomic<bool> detectedObject = false;
    // std::atomic<int> detectedClassIdx = 0;

    int detectedClassIdx = -1;

    std::vector<std::thread> threads;

    // Set maximum buffer size to 15
    const int max_buffer_size = 15;

    const std::vector<std::string> CLASS_NAMES = {
    "Black_Rectangle",
    "Red_Rectangle",
    "Blue_Rectangle",
    "Green_Rectangle",
    "White_Rectangle",
    "Purple_Rectangle",
    "Brown_Rectangle",
    "Orange_Rectangle",
    "Black_Circle",
    "Red_Circle",
    "Blue_Circle",
    "Green_Circle",
    "White_Circle",
    "Purple_Circle",
    "Brown_Circle",
    "Orange_Circle",
    "Black_Triangle",
    "Red_Triangle",
    "Blue_Triangle",
    "Green_Triangle",
    "White_Triangle",
    "Purple_Triangle",
    "Brown_Triangle",
    "Orange_Triangle",
    "Black_Semicircle",
    "Red_Semicircle",
    "Blue_Semicircle",
    "Green_Semicircle",
    "White_Semicircle",
    "Purple_Semicircle",
    "Brown_Semicircle",
    "Orange_Semicircle",
    "Black_Quartercircle",
    "Red_Quartercircle",
    "Blue_Quartercircle",
    "Green_Quartercircle",
    "White_Quartercircle",
    "Purple_Quartercircle",
    "Brown_Quartercircle",
    "Orange_Quartercircle",
    "Black_Pentagon",
    "Red_Pentagon",
    "Blue_Pentagon",
    "Green_Pentagon",
    "White_Pentagon",
    "Purple_Pentagon",
    "Brown_Pentagon",
    "Orange_Pentagon",
    "Black_Star",
    "Red_Star",
    "Blue_Star",
    "Green_Star",
    "White_Star",
    "Purple_Star",
    "Brown_Star",
    "Orange_Star",
    "Black_Cross",
    "Red_Cross",
    "Blue_Cross",
    "Green_Cross",
    "White_Cross",
    "Purple_Cross",
    "Brown_Cross",
    "Orange_Cross"};
    
    //const std::vector<std::string> CLASS_NAMES = {"Rectangle", "Circle", "Triangle", "Semicircle","Quartercircle","Pentagon","Star", "Cross"};

    const std::vector<std::vector<unsigned int>> COLORS = {
        {0, 114, 189},   {217, 83, 25},   {237, 177, 32},  {126, 47, 142},  {119, 172, 48},  {77, 190, 238},
        {162, 20, 47},   {76, 76, 76},    {153, 153, 153}, {255, 0, 0},     {255, 128, 0},   {191, 191, 0},
        {0, 255, 0},     {0, 0, 255},     {170, 0, 255},   {85, 85, 0},     {85, 170, 0},    {85, 255, 0},
        {170, 85, 0},    {170, 170, 0},   {170, 255, 0},   {255, 85, 0},    {255, 170, 0},   {255, 255, 0},
        {0, 85, 128},    {0, 170, 128},   {0, 255, 128},   {85, 0, 128},    {85, 85, 128},   {85, 170, 128},
        {85, 255, 128},  {170, 0, 128},   {170, 85, 128},  {170, 170, 128}, {170, 255, 128}, {255, 0, 128},
        {255, 85, 128},  {255, 170, 128}, {255, 255, 128}, {0, 85, 255},    {0, 170, 255},   {0, 255, 255},
        {85, 0, 255},    {85, 85, 255},   {85, 170, 255},  {85, 255, 255},  {170, 0, 255},   {170, 85, 255},
        {170, 170, 255}, {170, 255, 255}, {255, 0, 255},   {255, 85, 255},  {255, 170, 255}, {85, 0, 0},
        {128, 0, 0},     {170, 0, 0},     {212, 0, 0},     {255, 0, 0},     {0, 43, 0},      {0, 85, 0},
        {0, 128, 0},     {0, 170, 0},     {0, 212, 0},     {0, 255, 0},     {0, 0, 43},      {0, 0, 85},
        {0, 0, 128},     {0, 0, 170},     {0, 0, 212},     {0, 0, 255},     {0, 0, 0},       {36, 36, 36},
        {73, 73, 73},    {109, 109, 109}, {146, 146, 146}, {182, 182, 182}, {219, 219, 219}, {0, 114, 189},
        {80, 183, 189},  {128, 128, 0}};

    void capture_frames();
    void capture_frames_path(string path);
    void model_on();
    void model_on(string path);
    void model_off();
    void inference();

    std::vector<std::string> getClassNames();

    void setDetectedState(bool val);
    bool getDetectedState();
    void setDetectedClassIdx(int val);
    int getDetectedClassIdx();
};

#endif