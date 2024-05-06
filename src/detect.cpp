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

using namespace std;
using std::this_thread::sleep_for;

Detect::Detect() {

}

void Detect::capture_frames(){
    const std::string path = "rtsp://192.168.144.25:8554/main.264"; //PUT PATH HERE

    
    std::ofstream capLog;
    capLog.open("capLog.txt");

    if(!capLog.is_open()){
        std::cout << "File open failed! Ending program." << std::endl;
        return;
    }

    cout << "Starting RTSP Stream" << endl;
    capLog << "Starting RTSP Stream" << endl;

    cv::Mat             res, image;

    cv::VideoCapture cap(path);

    if (!cap.isOpened()) {
	    
        printf("can not open %s\n", path.c_str());
        capLog << "can not open " << path.c_str();
        return;
    }
    
    while (cap.read(image) && is_running) {
	    capLog << frame_buffer.size() << endl;

        // std::cout << "HELLO FROM CAPTUER FRAMES" << std::endl;

        // Lock the buffer before accessing it
        buffer_lock.lock();

        frame_buffer.push_back(image);

        // Remove elements from the front until buffer size is within limit
        while (frame_buffer.size() > max_buffer_size) {
            frame_buffer.pop_front();
        }

        // Unlock the buffer after accessing it
        buffer_lock.unlock();

        // std::cout << "FINISHED ITER IN CAPTURE FRAMES" << std::endl;

    }

    cap.release();
}

void Detect::capture_frames_path(string path){
    cout << "Opening capture stream from " << path << endl;
    cv::Mat             res, image;

    cv::VideoCapture cap(path);

    if (!cap.isOpened()) {
        printf("can not open %s\n", path.c_str());
        return;
    }
    
    while (cap.read(image) && is_running) {
        // Lock the buffer before accessing it
        buffer_lock.lock();

        // cout << "capturing frame" << endl;

        frame_buffer.push_back(image);

        // Remove elements from the front until buffer size is within limit
        while (frame_buffer.size() > max_buffer_size) {
            frame_buffer.pop_front();
        }

        // Unlock the buffer after accessing it
        buffer_lock.unlock();

        // cv::imshow("result", image);

        if (cv::waitKey(10) == 'q') {
            break;
        }

        // sleep_for(10ms);
    }

    cap.release();
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

    cout << "made it to model on" << endl;

    if (is_running) 
        return;

    cout << "made it past the fast return " << endl;

    is_running = true;

    std::thread capture_thread(&Detect::capture_frames_path, this, path);
    std::thread inference_thread(&Detect::inference, this);

    threads.push_back(std::move(capture_thread));
    threads.push_back(std::move(inference_thread));
}

void Detect::setDetectedState(bool val) {
    // detectedObject = true;
    return;
}

bool Detect::getDetectedState() {
    // return detectedObject;
    return true;
}

void Detect::setDetectedClassIdx(int val) {
    detectedClassIdx = val;
}

int Detect::getDetectedClassIdx() {

    int idx = 0;

    inference_lock.lock();

    idx = detectedClassIdx;

    inference_lock.unlock();

    return idx;

}


// void checkObjectDetected() {
//     return 
// }

void Detect::inference(){
    const std::string engine_file_path = "/home/buckeyevertical/Documents/YOLOv8-TensorRT/best.engine"; //PUT PATH HERE

    std::vector<std::string> imagePathList;

    auto yolov8 = new YOLOv8(engine_file_path);
    yolov8->make_pipe(true);

    std::vector<Object> objs;
    cv::Size            size = cv::Size{1280, 1280}; //Change Here


    const int VIDEO_FRAME_RATE = 10;

    // Define the codec and create VideoWriter object
    cv::VideoWriter video("output.mp4", cv::VideoWriter::fourcc('M','J','P','G'), VIDEO_FRAME_RATE, cv::Size{1280, 720}, true);

    // Check if VideoWriter opened successfully=
    if (!video.isOpened()) {
        std::cerr << "Error: Unable to open VideoWriter" << std::endl;
        return;
    }

    std::ofstream infLog;
    infLog.open("infLog.txt");

    if(!infLog.is_open()){
        std::cout << "infLog.txt File open failed! Ending program." << std::endl;
        return;
    }

    infLog << "Opened log file" << endl;

    auto startTime = std::chrono::system_clock::now();

    while(is_running){
	    infLog << "Running Inference..." << endl;

        // std::cout << ":::::FROM INFERENCE" << std::endl;

        // Lock the buffer before accessing it
        buffer_lock.lock();

        // Get the last frame
        if (frame_buffer.size() > 0) {

            // gets last element of the buffer
            cv::Mat image = frame_buffer.at(frame_buffer.size() - 1);

            // frame_buffer.push_back(image);

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
            // std::cout << "objs size " << objs.size() << std::endl;

            // if (!detectedObject) {

            //     if (objs.size() > 0) {

            //         inference_lock.lock();

            //         detectedClassIdx = objs.at(0).label;
            //         // detectedObject = true;

            //         inference_lock.unlock();

            //     }

            // }


            inference_lock.lock();

            detectedClassIdx = -1;

            // Get the end time
            auto current = std::chrono::system_clock::now();

            // Calculate the duration
            std::chrono::duration<double> elapsed_seconds = current - startTime;

    
            if (objs.size() > 0){
                detectedClassIdx = objs.at(0).label;
                cout << "Detected " << detectedClassIdx << " Object at " << elapsed_seconds.count() << endl;
                infLog << "Detected " << detectedClassIdx << " Object at " << elapsed_seconds.count() << endl;
            }

            inference_lock.unlock();



            //cv::imshow("result", image);

            //cout << image.size() << endl;
            video.write(image);
        

            if (cv::waitKey(1) == 'q') {
                break;
            }
        } else {
            buffer_lock.unlock();
            std::cout << "Frame buffer is empty!" << std::endl;
            sleep_for(1000ms);
        }

        // std::cout << ":::::FINISHED INFERENCE" << std::endl;
    }

    infLog.close();
    video.release();
    cv::destroyAllWindows();
    delete yolov8;
}

int Detect::getClassIdx(string name) {
    for (int i = 0; i < CLASS_NAMES.size(); i++ ) {
        if (name == CLASS_NAMES.at(i))
            return i;
    }
}

// int main(int argc, char** argv)
// {
//     std::thread capture_thread(capture_frames);

//     std::thread inference_thread(inference);

//     inference_thread.join();
//     capture_thread.join();

//     return 0;
// }
