#pragma once

class VideoSource {
public:
    virtual const cv::Mat& captureFrame() = 0;
    // the point during captureFrame() at which the actual state of the underlying image is captured
    // (accounting for memory transfer etc.)
    virtual double capturePoint() const = 0;
};
