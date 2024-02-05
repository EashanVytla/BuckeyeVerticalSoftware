#pragma once

#include <condition_variable> 
#include <iostream> 
#include <mutex> 
#include <queue> 
  
using namespace std;
 
template <class T> 
class SafeQueue { 
    private: 
        queue<T> q; 
        mutex mut;  
        condition_variable cond; 
    
    public: 
        void enqueue(T element);
        T dequeue(); 
        size_t size(); 
};

// Definitions

template <class T> 
void SafeQueue<T>::enqueue(T element) { 

    unique_lock<mutex> lock(mut); 

    q.push(element); 
    cond.notify_one(); 
};

template <class T> 
T SafeQueue<T>::dequeue() { 

    unique_lock<mutex> lock(mut); 

    cond.wait(lock, [this]() { return !q.empty(); }); 

    T element = q.front(); 
    q.pop(); 

    return element; 

} 

template <class T> 
size_t SafeQueue<T>::size() { 

    unique_lock<mutex> lock(mut); 

    return q.size(); 

} 
// TODO: add empty() if needed

