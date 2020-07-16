#include <iostream>
#include <tools.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

//klt tracker
void print_corners(cv::Mat & image, vector<cv::Point2f> & point_vec){
	for (auto & element : point_vec){
     	cv::circle(image, element, 3, 255, -1);
     }
}

int main(void) {
     //klt tracker

     string video_path = "/home/administrator/slam_test/visual_slam/src/slow_traffic_small.mp4";
     cv::VideoCapture capt;
     capt.open(video_path);
     if (!capt.isOpened()) {
    	cerr << "ERROR! Unable to open video\n";
     	return -1;
    }

    cv::Mat frame1, frame2, frame1_gray, frame2_gray;
    capt.read(frame1);
    for (int j = 0;j<50;j++){
    	capt.read(frame2);
    }
    cv::imshow("frame1",frame1);
    cv::waitKey(0);
    cv::imshow("frame2",frame2);
    cv::waitKey(0);

    cvtColor(frame1, frame1_gray, cv::COLOR_BGR2GRAY);
    cvtColor(frame2, frame2_gray, cv::COLOR_BGR2GRAY);

    vector<cv::Point2f> p1;
    vector<cv::Point2f> p2;
    cv::goodFeaturesToTrack(frame1_gray, p1, 500, 0.05, 7, cv::Mat(), 7, true, 0.04);
    cv::goodFeaturesToTrack(frame2_gray, p2, 500, 0.3, 7, cv::Mat(), 7, true, 0.04);

	
	for (uint i =0; i<p1.size();i++){
		cv::circle(frame1, p1[i], 5 , (255,0,0), -1);
	}
    cv::imshow("Frame", frame1);
    cv::waitKey(0);


    cv::Mat mask = cv::Mat::zeros(frame1.size(), frame1.type());
    vector<cv::Scalar> colors;
    cv::RNG rng;
    for(int i = 0; i < 100; i++)
    {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors.push_back(cv::Scalar(r,g,b));
    }
    // calculate optical flow
    vector<uchar> status;
    vector<float> err;
    cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
    cv::calcOpticalFlowPyrLK(frame1_gray, frame2_gray, p1, p2, status, err, cv::Size(15,15), 2, criteria);
    vector<cv::Point2f> good_new;
    for(uint i = 0; i < p1.size(); i++)
    {
        // Select good points
        if(status[i] == 1) {
            good_new.push_back(p1[i]);
            // draw the tracks
            cv::line(mask,p2[i], p1[i], colors[i], 2);
            cv::circle(frame1, p2[i], 5, colors[i], -1);
        }
    }
    cv::Mat img;
    cv::add(frame1, mask, img);
    cv::imshow("Frame", img);
    cv::waitKey(0);

    /*vector<vector<float>> v(3);
    cv::convertPointsToHomogeneous(p1, v);
    cout<<v[0][0]<<v[0][1]<<v[0][2];*/

    cv::Mat F_mat = cv::findFundamentalMat(p1,p2,cv::FM_RANSAC,3,0.99);
    F_mat.convertTo(F_mat, CV_32FC1);
    cout<<F_mat<<endl<<p1[0]<<endl<<p2[0]<<endl;
    //float a[3];
    vector<cv::Point3f> p1_h(p1.size()), p2_h(p2.size()) ;

    cv::convertPointsToHomogeneous(p1, p1_h);
    cv::convertPointsToHomogeneous(p2, p2_h);
    cout<<p2_h[0]<<endl;
    cout<<cv::Mat(p2_h[0]).t()<<endl;
    for(int k = 0; k<36;k++){
    	cv::Mat_ <double> mult = cv::Mat(p1_h[k]).t() * F_mat * cv::Mat(p2_h[k]);
    	cout<<endl<<mult;
    }

    /*float res = p1[1]*F_mat;
    res = res * p2[1];
    cout << res;*/




    
    //int keyboard = cv::waitKey(30);
    /*if (keyboard == 'q' || keyboard == 27)
        break;*/
    // Now update the previous frame and previous points
    frame1_gray = frame1_gray.clone();
    p1 = good_new;

    /*for(int i = 0; i<100; i++){

    }*/
     return(0);
    

}
