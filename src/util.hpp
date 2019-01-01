#pragma once

#include <iostream>
#include <opencv2/core/types.hpp>

// no need for endl in cerr: https://www.youtube.com/watch?v=GMqQOEZYVJQ
//TODO do better - prevent expr from even being evaluated in release
#ifndef NDEBUG
#define WARN_UNLESS(expr, msg)     \
    if (!(expr)) {                 \
        std::cerr << msg << "\n";  \
    }
#else
#define WARN_UNLESS(expr, msg)
#endif

// BGR
const cv::Scalar CV_RED{0, 0, 255};
const cv::Scalar CV_BLUE{255, 0, 0};
const cv::Scalar CV_CYAN{255, 255, 0};