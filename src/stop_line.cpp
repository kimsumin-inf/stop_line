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
    cameraMatrix = (Mat1d(3,3)<< 7.480117180553555e+02,0., 6.438774178004406e+02, 0. ,7.474460107834711e+02, 3.663461209630810e+02, 0., 0. ,1);
    distCoeffs= (Mat1d(1,5)<< 0.185901934907552, -0.462158211869201 ,0., 0.,  0.273177418880758);
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix,Size(1280,720),CV_32FC1, map1, map2);
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
    base_frame = calibrated(base_frame);
    base_frame = return_byv(base_frame);
    Mat hls_frame;
    cvtColor(base_frame, hls_frame, COLOR_BGR2HLS);
    vector<Mat> hls_images(3);
    split(hls_frame, hls_images);
    Mat l_frame = hls_images[1];
    medianBlur(l_frame, l_frame, 7);
    return_thresh(low, high, 0.33, medianMat(l_frame));
    Mat bin_frame;
    threshold(l_frame,bin_frame, 250, 255, THRESH_BINARY);
    Mat sobel_frame;
    Sobel(bin_frame, sobel_frame,-1, 0,1);
    vector<Vec4i> lines_bin;

    HoughLinesP(sobel_frame, lines_bin, 1, CV_PI/180, 10 ,100, 10);

    Mat lines(base_frame.rows, base_frame.cols, CV_8UC3, Scalar(0,0,0));
    points.clear();
    for (auto i : lines_bin){
        if (calc_theta(Point(i[0],i[1]), Point(i[2],i[3]))<3  ){
            points.push_back(Point(i[0],i[1]));
            points.push_back(Point(i[2],i[3]));
        }
    }
    for (auto i: points){
        circle(base_frame, i,3, Scalar(0,0,255), 2, 8);
    }
    Vec4f line_para;
    if (points.size()>3) {
        fitLine(points, line_para, DIST_L2, 0, 1e-2, 1e-2);
        cout << "line_para: " << line_para << endl;


        Point pt0;
        pt0.x = line_para[2];
        pt0.y = line_para[3];
        double k = line_para[1] / line_para[0];

        Point pt1, pt2;
        pt1.x = 0;
        pt1.y = k * (0 - pt0.x) + pt0.y;
        pt2.x = 1280;
        pt2.y = k * (1280 - pt0.x) + pt0.y;

        line(base_frame, pt1, pt2, Scalar(0, 255, 0), 2, 8, 0);
        base_frame= return_rebyv(base_frame);
        show("base_frame", base_frame, 1);


    }
    else {
        base_frame= return_rebyv(base_frame);
        show("base_frame", base_frame, 1);
    }




    clear();

}
Mat SLD::return_gray(cv::Mat Input) {
    Mat tmp;
    cvtColor(Input, tmp, COLOR_BGR2GRAY);
    return tmp;
}
Mat SLD::return_gaussian_filter(cv::Mat frame) {
    Mat tmp;
    GaussianBlur(frame,tmp, Size(3,3),0);
    return tmp;
}
inline double SLD::calc_theta(cv::Point pt1, cv::Point pt2) {
    return abs(atan2(abs(pt1.y-pt2.y),abs(pt1.x-pt2.x)))*180/CV_PI;
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
    Mat tmp;
    Canny(frame,tmp, low, high);
    return tmp;
}
Mat SLD::calibrated(Mat frame){
    Mat tmp ; 
    remap(frame, tmp, map1, map2, INTER_LINEAR);
    return tmp;
}
Mat SLD::return_byv(cv::Mat frame) {
    int width  = frame.cols;
    int height = frame.rows;
    vector<Point2f> ps(4);
    ps[0] = Point2f(width/2 - 200, 20);
    ps[1] = Point2f(width/2 + 200, 20);
    ps[2] = Point2f(-880, height);
    ps[3] = Point2f(2160, height);

    vector<Point2f> pd(4);
    pd[0] = Point2f(0,0);
    pd[1] = Point2f(width,0);
    pd[2] = Point2f(0, height);
    pd[3] = Point2f(width, height);


    Mat perspective = getPerspectiveTransform(ps, pd);
    Mat tmp;
    warpPerspective(frame, tmp, perspective, Size(width, height), INTER_LINEAR);
    return tmp;

}
Mat SLD::return_rebyv(cv::Mat frame) {
    int width  = frame.cols;
    int height = frame.rows;
    vector<Point2f> ps(4);
    ps[0] = Point2f(width/2 - 200, 20);
    ps[1] = Point2f(width/2 + 200, 20);
    ps[2] = Point2f(-880, height);
    ps[3] = Point2f(2160, height);

    vector<Point2f> pd(4);
    pd[0] = Point2f(0,0);
    pd[1] = Point2f(width,0);
    pd[2] = Point2f(0, height);
    pd[3] = Point2f(width, height);


    Mat perspective = getPerspectiveTransform(pd, ps);
    Mat tmp;
    warpPerspective(frame, tmp, perspective, Size(width, height), INTER_LINEAR);
    return tmp;

}
double SLD::euclidean_distance(cv::Point pt1, cv::Point pt2) {
    return sqrt(pow((pt2.x- pt1.x),2)+ pow((pt2.y-pt1.y),2));
}
void SLD::show(std::string frame_name, cv::Mat frame, int waitkey=1) {
    imshow(frame_name, frame);
    waitKey(waitkey);
}

void SLD::clear(){
    printf("\033[2J");
    printf("\033[1;1H");
}