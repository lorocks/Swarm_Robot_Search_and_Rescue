#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

int function1(int input)
{
  cv::Mat img;
  img = cv::imread("test.png", cv::IMREAD_GRAYSCALE); 
  fprintf (stderr, "Calling OpenCV function\n");
  
  return input + 100;
}
