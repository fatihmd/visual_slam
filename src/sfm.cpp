#include <iostream>
#include <tools.h>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;

//klt tracker

int main(void) {

     cout << "Hello World" << endl;
     //klt tracker
     //vector of feature points
     vector<cv::Point2f> p0;
     //vector of images
     vector<cv::Mat> image_seq;
     //test image
     cv::Mat test_im = cv::imread("/home/administrator/slam_test/visual_slam/src/hamsi.png", cv::IMREAD_GRAYSCALE);
     cv::imshow("test",test_im);
     cv::waitKey(0);
     cv::goodFeaturesToTrack(test_im, p0, 500, 0.3, 7, cv::Mat(), 7, true, 0.04);
     // Create a mask image for drawing purposes
     cv::Mat mask = cv::Mat::zeros(test_im.size(), test_im.type());
     cout<< p0[0];
     int x,y;
     for (auto & element : p0){
     	cv::circle(test_im, element, 3, 255, -1);
     }
     cv::imshow("test",test_im);
     cv::waitKey(0);
     return(0);

}
