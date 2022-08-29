#ifndef FLAPPYBIRD_SCREEN_CAPTURE_HPP
#define FLAPPYBIRD_SCREEN_CAPTURE_HPP

#include "VideoSource.hpp"

#include <opencv2/opencv.hpp>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <vector>

class ScreenCapture : public VideoSource {

public:
const cv::Mat& captureFrame() override {
    Display* display = XOpenDisplay(nullptr);
    Window root = DefaultRootWindow(display);

    XWindowAttributes attributes = {0};
    XGetWindowAttributes(display, root, &attributes);

    const int width = 1055;//attributes.width;
    const int height = 1184;//attributes.height;

    XImage* img = XGetImage(display, root, 464, 436 , width, height, AllPlanes, ZPixmap);
    const int bitsPerPixel = img->bits_per_pixel;

    m_pixelBuffer.resize(width * height * 4);

    memcpy(&m_pixelBuffer[0], img->data, m_pixelBuffer.size());

    XDestroyImage(img);
    XCloseDisplay(display); //TODO necessary?

    m_currentFrame = cv::Mat(height, width, bitsPerPixel > 24 ? CV_8UC4 : CV_8UC3, &m_pixelBuffer[0]);
    return m_currentFrame;
}

private:
    cv::Mat m_currentFrame;
    std::vector<uint8_t> m_pixelBuffer;
};

#endif //FLAPPYBIRD_SCREEN_CAPTURE_HPP
