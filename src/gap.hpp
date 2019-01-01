#pragma once

#include "opencv2/core/types.hpp"

struct Gap {
    cv::Point lowerLeft;
    cv::Point lowerRight;
    cv::Point upperLeft;
    cv::Point upperRight;
};
