//
// Created by sumin on 22. 8. 24.
//
#include "stop_line/stop_line.h"

int main(int argc , char** argv){
    ros::init(argc, argv, "stop_line");
    SLD sld;
    ros::spin();
    return 0;
}