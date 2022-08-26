//
// Created by sumin on 22. 8. 24.
//

#ifndef SRC_STOP_LINE_H
#define SRC_STOP_LINE_H

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <string>
#include <vector>

#include <sensor_msgs/Image.h>
#include <std_msgs/Int16.h>


class SLD{
private:
    void cam_CB(const sensor_msgs::Image::ConstPtr &msg);
    void steer_CB(const std_msgs::Int16::ConstPtr& msg);
    cv::Mat return_gray(cv::Mat Input);
    cv::Mat return_gaussian_filter(cv::Mat frame);
    double medianMat(cv::Mat Input);
    void return_thresh(int &low, int &high, double sigma, double mid);
    cv::Mat return_Canny(cv::Mat frame, int low, int high);
    cv::Mat return_byv(cv::Mat frame );

    double euclidean_distance(cv::Point pt1, cv::Point pt2);
    inline double calc_theta(cv::Point pt1, cv::Point pt2);
    cv::Mat calibrated(cv::Mat frame);
    void show(std::string frame_name, cv::Mat frame, int waitkey);
    void clear();

    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Subscriber subCAM;
    ros::Subscriber subSteer;
    cv::Mat base_frame;
    cv::Mat gray_frame;
    cv::Mat canny_frame;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat map1, map2;
    cv_bridge::CvImagePtr cv_ptr;
    double middle_value;
    int low, high;
    int steer;


public:
    SLD();


};


#endif //SRC_STOP_LINE_H
