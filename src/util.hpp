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

//TODO figure this out, learn about ulps
inline bool approxEqual(float a, float b, float e = 0.e-4f) {
    return std::abs(a / b) < e;
}

inline TimePoint toTime(std::chrono::system_clock::time_point&& point) {
    return std::chrono::time_point_cast<TimePoint::duration>(point);
}

class RAIICloser {
public:
    RAIICloser(std::function<void()> closer) : m_closer(closer) {}
    ~RAIICloser() {
        m_closer();
    }
private:
    std::function<void()> m_closer;
};
