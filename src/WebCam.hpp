#ifndef FLAPPYBIRD_WEBCAM_HPP
#define FLAPPYBIRD_WEBCAM_HPP

#include <opencv2/videoio.hpp>
#include "VideoSource.hpp"

class WebCam : public VideoSource {

public:
    const cv::Mat& captureFrame() override {
        if (!m_cap.isOpened()) { // if not successful, exit program
            throw std::logic_error{"Cannot capture video: VideoCapture not open"};
        }

        cv::Mat temp;
        if (!m_cap.read(temp)) { // read a new frame from camera
            throw std::runtime_error{"Cannot read a frame from video stream"};
        }

        cv::flip(temp, m_currentFrame, -1);

        return m_currentFrame;
    }

    double capturePoint() const override{
        return WEBCAM_CAPTURE_POINT;
    }

private:
    cv::VideoCapture m_cap{-1};
    cv::Mat m_currentFrame;
};

#endif //FLAPPYBIRD_WEBCAM_HPP
