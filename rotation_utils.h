// rotation_utils.hpp
#ifndef ROTATION_UTILS_HPP
#define ROTATION_UTILS_HPP

#include <opencv2/core.hpp>

cv::Mat Rotx(double theta_deg);
cv::Mat Roty(double theta_deg);
cv::Mat Rotz(double theta_deg);

#endif  // ROTATION_UTILS_HPP