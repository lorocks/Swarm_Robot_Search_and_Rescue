#ifndef SEARCH_HPP
#define SEARCH_HPP

#include <iostream>
#include <opencv2/opencv.hpp>

class ObjectSearch {
public:
    ObjectSearch(const std::string& modelPath, const std::string& configPath);
    ~ObjectSearch();

    // Function to initialize the object detection model (YOLO or any other)
    void initializeObjectDetection(const std::string& modelPath, const std::string& configPath);

    // Function to analyze the video stream and run object detection
    bool runObjectDetection(const cv::Mat& frame);

    // Function to check if the object is found
    bool isObjectFound() const;

private:
    // Add any private members or helper functions as needed
    bool objectFound;
    cv::dnn::Net humanDetectionModel;
};

#endif // SEARCH_HPP


