#include "search.hpp"

ObjectSearch::ObjectSearch(const std::string& modelPath, const std::string& configPath)
    : objectFound(false) {
    // Initialize the model and configuration paths
    initializeObjectDetection(modelPath, configPath);
}

ObjectSearch::~ObjectSearch() {
    // Destructor implementation (if needed)
}

void ObjectSearch::initializeObjectDetection(const std::string& modelPath, const std::string& configPath) {
    // Implement code to initialize the object detection model (YOLO or any other)
    // Use modelPath and configPath for loading the YOLO model and configuration
    
    humanDetectionModel = cv::dnn::readNet(modelPath, configPath);
    
}

bool ObjectSearch::runObjectDetection(const cv::Mat& frame) {
    // Implement code to analyze the video stream using the object detection model
    // Update the internal state (e.g., set the boolean to true if the object is found)
    // Return the boolean value indicating whether the object is found or not
    

    
    // Process the detection results and update the objectFound variable accordingly


    return objectFound;
}

bool ObjectSearch::isObjectFound() const {
    return objectFound;
}
