#pragma once

#include "opencv2/videoio.hpp"

#include "gap.hpp"

class Camera {
public:
    Camera();
    virtual ~Camera();

    void capture(cv::Point& birdPos, cv::Mat& world);
    void mark(const cv::Point & loc, const cv::Scalar& color);
    void circle(const cv::Point& center, int radius, const cv::Scalar& color);
    void markGaps(const Gap& leftGap, const Gap& rightGap);
    
    void mouseClick(int x, int y);
    
    unsigned getAbsoluteHeight(float proportionOfScreenHeight) {
        assert(boundariesKnown);
        return proportionOfScreenHeight * m_screenHeight;
    }

    inline unsigned getScreenWidth() {
        assert(boundariesKnown);
        return m_screenWidth;
    }

    inline unsigned getGroundLevel() {
        assert(boundariesKnown);
        return m_bottomLeftBoundary.y;
    }

    inline unsigned getRightBoundary() {
        assert(boundariesKnown);
        return m_bottomRightBoundary.x;
    }

    bool boundariesKnown;
    void drawLine(const cv::Point & a, const cv::Point& b, const cv::Scalar& color);
private:
    void saveBoundaries() const;
    void loadBoundaries();

    bool m_showFeed;
    const std::string m_feedName;
    cv::VideoCapture m_cap;
    cv::Mat m_currentFrame;

    cv::Point m_clicks[3];
    size_t m_currentClick;
    cv::Point m_bottomLeftBoundary;
    cv::Point m_bottomRightBoundary;
    unsigned m_screenHeight;
    unsigned m_screenWidth;
};
