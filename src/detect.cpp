// #pragma once

//
// Created by ubuntu on 3/16/23.
//

#include "yolov8.hpp"
#include "detect.h"

// #include "chrono"
// #include "opencv2/opencv.hpp"
// #include <iostream>
// #include <deque>
// #include <mutex>
// #include <thread>
// #include <atomic>

Detect::Detect() {

}

void Detect::capture_frames(){
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

void Detect::capture_frames(string path){
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

void Detect::model_off(){
    is_running = false;

    for (auto&& t : threads) {
        if (t.joinable()) {
            t.join();
        }

    }
    
}

std::vector<std::string>  Detect::getClassNames() {
    return CLASS_NAMES;
}

void Detect::model_on(){

    if (is_running) 
        return;

    is_running = true;

    std::thread capture_thread(&Detect::capture_frames, this);
    std::thread inference_thread(&Detect::inference, this);

    threads.push_back(std::move(capture_thread));
    threads.push_back(std::move(inference_thread));
}

void Detect::model_on(string path){

    if (is_running) 
        return;

    is_running = true;

    std::thread capture_thread(&Detect::capture_frames, this, path);
    std::thread inference_thread(&Detect::inference, this);

    threads.push_back(std::move(capture_thread));
    threads.push_back(std::move(inference_thread));
}

void Detect::setDetectedState(bool val) {
    detectedObject = true;
}

bool Detect::getDetectedState() {
    return detectedObject;
}

void Detect::setDetectedClassIdx(int val) {
    detectedClassIdx = val;
}

int Detect::getDetectedClassIdx() {
    return detectedClassIdx;
}


// void checkObjectDetected() {
//     return 
// }

void Detect::inference(){
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

            // TO MAKE SURE DETECTEDCLASSIDX DOESN'T CHANGE
            if (!detectedObject) {
                detectedClassIdx = objs.at(0).label;
                detectedObject = true;
            }

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