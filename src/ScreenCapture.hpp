#ifndef FLAPPYBIRD_SCREEN_CAPTURE_HPP
#define FLAPPYBIRD_SCREEN_CAPTURE_HPP

#include "VideoSource.hpp"

#include <opencv2/opencv.hpp>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <vector>

class ScreenCapture : public VideoSource {

public:
    ScreenCapture(Display* const x11display) : m_x11display(x11display) {}
    const cv::Mat& captureFrame() override {
        // this whole function takes around 5ms, most of it in XGetImage

        Window root = DefaultRootWindow(m_x11display);

        // viewport into the Android emulator, screen coordinates can be found using:
        // cnee --record --mouse | awk  '/7,4,0,0,1/ { system("xdotool getmouselocation") }'
        // remember to set viewport boundaries into that viewport (like we would with a webcam, see display.cpp)
        const int leftX = 565;
        const int rightX = 1419;
        const int topY = 745;
        const int bottomY = 1708;
        const int width = rightX - leftX;
        const int height = bottomY - topY;
        XImage* img = XGetImage(m_x11display, root, leftX, topY, width, height, AllPlanes, ZPixmap);
        const int bitsPerPixel = img->bits_per_pixel;

        m_pixelBuffer.resize(width * height * 4);

        memcpy(&m_pixelBuffer[0], img->data, m_pixelBuffer.size());

        XDestroyImage(img);

        m_currentFrame = cv::Mat(height, width, bitsPerPixel > 24 ? CV_8UC4 : CV_8UC3, &m_pixelBuffer[0]);
        return m_currentFrame;
    }

    double capturePoint() const override{
        return CAPTURE_POINT;
    }

private:
    Display* const m_x11display;
    cv::Mat m_currentFrame;
    std::vector<uint8_t> m_pixelBuffer;
};

#endif //FLAPPYBIRD_SCREEN_CAPTURE_HPP
