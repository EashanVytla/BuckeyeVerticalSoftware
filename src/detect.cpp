//
// Created by ubuntu on 3/16/23.
//
#include "chrono"
#include "opencv2/opencv.hpp"
#include "yolov8.hpp"
#include <iostream>
#include <deque>
#include <mutex>
#include <thread>
#include <atomic>



std::deque<cv::Mat> frame_buffer;
std::mutex buffer_lock;
std::atomic<bool> is_running(true);

std::atomic<bool> detectedObject(false);
std::atomic<int> detectClassIdx(0);

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


void capture_frames(){
    const std::string path = "rtsp://192.168.144.25:8554/main.264"; //PUT PATH HERE

    cv::Mat             res, image;

    cv::VideoCapture cap(path);

    if (!cap.isOpened()) {
        printf("can not open %s\n", path.c_str());
        return;
    }
    
    while (cap.read(image) && is_running) {
        // Lock the buffer before accessing it
        buffer_lock.lock();

        frame_buffer.push_back(image);

        // Remove elements from the front until buffer size is within limit
        while (frame_buffer.size() > max_buffer_size) {
            frame_buffer.pop_front();
        }

        // Unlock the buffer after accessing it
        buffer_lock.unlock();
    }
}

void model_off(){
    is_running = false;

    for (int i = 0; i < threads.size(); i++) 
        threads.at(i).join();

    
}


void model_on(){

    if (is_running) 
        return;

    is_running = true;

    std::thread capture_thread(capture_frames);
    std::thread inference_thread(inference);

    threads.push_back(capture_thread);
    threads.push_back(inference_thread);

}

// void checkObjectDetected() {
//     return 
// }

void inference(){
    const std::string engine_file_path = "/home/buckeyevertical/Documents/YOLOv8-TensorRT/Feb29RL.engine"; //PUT PATH HERE

    std::vector<std::string> imagePathList;

    auto yolov8 = new YOLOv8(engine_file_path);
    yolov8->make_pipe(true);

    std::vector<Object> objs;
    cv::Size            size = cv::Size{1280, 1280}; //Change Here

    while(is_running){
        // Lock the buffer before accessing it
        buffer_lock.lock();

        // Get the last frame
        if (!frame_buffer.empty()) {
            cv::Mat image = frame_buffer.back();

            frame_buffer.push_back(image);



            // Unlock the buffer after accessing it
            buffer_lock.unlock();

            yolov8->copy_from_Mat(image, size);
            auto start = std::chrono::system_clock::now();
            yolov8->infer();
            auto end = std::chrono::system_clock::now();
            yolov8->postprocess(objs);
            yolov8->draw_objects(image, image, objs, CLASS_NAMES, COLORS);
            auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;


            detectClassIdx = objs.at(0).label;
            detectedObject = true;


            cv::imshow("result", image);

            if (cv::waitKey(1) == 'q') {
                break;
            }
        } else {
            std::cout << "Frame buffer is empty!" << std::endl;
            buffer_lock.unlock();
        }
    }

    cv::destroyAllWindows();
    delete yolov8;
}

// int main(int argc, char** argv)
// {
//     std::thread capture_thread(capture_frames);

//     std::thread inference_thread(inference);

//     inference_thread.join();
//     capture_thread.join();

//     return 0;
// }