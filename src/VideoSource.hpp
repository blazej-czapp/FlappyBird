#pragma once

class VideoSource {
public:
    virtual const cv::Mat& captureFrame() = 0;

    // assuming the real frame is captured immediately at captureFrame(), how much time does processing and data
    // transfer take?
    virtual std::chrono::milliseconds postCaptureProcessingTime() const = 0;
};
