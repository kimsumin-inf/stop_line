//
// Created by sumin on 22. 8. 24.
//
#include "stop_line/stop_line.h"

using namespace std;
using namespace cv;

SLD::SLD()
:nh(""), pnh("")
{
    subCAM = nh.subscribe("/camera_front/image_raw", 1, &SLD::cam_CB, this);
    subSteer = nh.subscribe("/MSG_CON/Rx_Steer", 1,&SLD::steer_CB, this);

}
void SLD::steer_CB(const std_msgs::Int16::ConstPtr &msg) {
    steer= msg->data;
}
void SLD::cam_CB(const sensor_msgs::Image::ConstPtr &msg) {
    ROS_INFO("Cam CallBack Success");
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        base_frame = cv_ptr->image;
    }catch(cv_bridge::Exception &e){
        ROS_INFO("Error to Convert");
        return ;
    }

    gray_frame = return_gray(base_frame);
    middle_value = medianMat(gray_frame);
    return_thresh(low, high, 0.2, middle_value);
    medianBlur(gray_frame,gray_frame, 15);
    canny_frame = return_Canny(gray_frame, low, high);
    vector<Vec4i> linesP;

    HoughLinesP(canny_frame, linesP,1, CV_PI/180.,15,30,50 );
    Mat lines(canny_frame.rows, canny_frame.cols,CV_8UC3, Scalar(0,0,0));

    for (auto i : linesP){
        Vec4i l = i;

        if (abs(steer/71)<5){
            if (abs(atan2(l[3]-l[1], l[2]-l[0])*180/CV_PI)<=5 ) {
                if (euclidean_distance(Point(l[0],l[1]), Point(l[2],l[3]))>=200) {
                    cout <<atan2(l[3]-l[1], l[2]-l[0])*180/CV_PI<<endl;
                    cout <<  euclidean_distance(Point(l[0],l[1]), Point(l[2],l[3])) <<endl;
                    line(lines, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, LINE_AA);
                    line(base_frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, LINE_AA);
                    cout <<"______________"<<endl;
                }
            }
            else if (atan2(l[3]-l[1], l[2]-l[0])*180/CV_PI>=80 && atan2(l[3]-l[1], l[2]-l[0])*180/CV_PI<=100 ){
                line(lines, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 1, LINE_AA);
                line(base_frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 1, LINE_AA);
            }
            cout <<"______________"<<endl;

            }
    }

    show("origin", base_frame, 1);
    show("canny", canny_frame, 1);
    show("line", lines,1);

    clear();

}
Mat SLD::return_gray(cv::Mat Input) {
    cvtColor(Input, Input, COLOR_BGR2GRAY);
    return Input;
}
Mat SLD::return_gaussian_filter(cv::Mat frame) {
    GaussianBlur(frame,frame, Size(3,3),0);
    return frame;
}

double SLD::medianMat(Mat Input) {
    Input = Input.reshape(0,1);
    vector<double> vecFromMat;
    Input.copyTo(vecFromMat);
    nth_element(vecFromMat.begin(), vecFromMat.begin()+vecFromMat.size()/2, vecFromMat.end());
    return vecFromMat[vecFromMat.size()/2];
}

void SLD::return_thresh(int &low, int &high, double sigma, double mid) {
    if ((1.0-sigma)*mid > 0){
        low = (1.0-sigma)*mid;
    }
    else {
        low = 0;
    }
    if((1.0+sigma)*mid<255){
        high =(1.0+sigma)*mid;
    }
    else {
        high = 250;
    }
    ROS_INFO("low: %d, high: %d, mid: %f", low, high, mid);
}

Mat SLD::return_Canny(cv::Mat frame, int low, int high) {
    Canny(frame,frame, low, high);
    return frame;
}
double SLD::euclidean_distance(cv::Point pt1, cv::Point pt2) {
    return sqrt(pow((pt2.x- pt1.x),2)+ pow((pt2.y-pt1.y),2));
}
void SLD::show(std::string frame_name, cv::Mat frame, int waitkey) {
    imshow(frame_name, frame);
    waitKey(waitkey);
}

void SLD::clear(){
    printf("\033[2J");
    printf("\033[1;1H");
}