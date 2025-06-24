// rotation_utils.cpp
#include "rotation_utils.h"
#include <cmath>
#include <iostream>

cv::Mat Rotx(double theta_deg) {
    double theta = theta_deg * CV_PI / 180.0;
    cv::Mat Rx = (cv::Mat_<double>(3, 3) <<
        1, 0, 0,
        0, cos(theta), -sin(theta),
        0, sin(theta), cos(theta)
    );
    return Rx;
}

cv::Mat Roty(double theta_deg) {
    double theta = theta_deg * CV_PI / 180.0;
    cv::Mat Ry = (cv::Mat_<double>(3, 3) <<
        cos(theta), 0, sin(theta),
        0,          1,          0,
        -sin(theta), 0, cos(theta)
    );
    return Ry;
}

cv::Mat Rotz(double theta_deg) {
    double theta = theta_deg * CV_PI / 180.0;
    cv::Mat Rz = (cv::Mat_<double>(3, 3) <<
        cos(theta), -sin(theta), 0,
        sin(theta),  cos(theta), 0,
        0,                   0, 1
    );
    return Rz;
}
