#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <deque>
#include <opencv2/core.hpp>


struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
};

template <typename T>
class RingBuffer : public std::deque<T> {
private:
    std::deque<T> c;
    int maxSize;
public:
    RingBuffer(int size) : maxSize(size) {
        this->c.clear();
    }
    void push(const T& value) {
        if(this->c.size() >= this->maxSize) {
            this->c.pop_front();
        }
        this->c.push_back(value);
    }
    typename std::deque<T>::iterator end() {
        return this->c.end();
    }
    size_t size() {
        return c.size();
    }
};

#endif /* dataStructures_h */
