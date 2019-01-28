#pragma once

#include <functional>

#include "opencv2/videoio.hpp"

#include "gap.hpp"

class Display {
    static const std::string FEED_NAME;

public:
    Display();
    virtual ~Display();

    void capture();
    /// capture() equivalent but takes frame as input rather than from the actual camera
    void playback(cv::Mat& recordedFrame);
    void show() const;

    /**
     *
     * @param[in] frame
     * @param[out] birdPos
     * @param[out] thresholdedWorld - thresholded, black&white image of just the obstacles
     */
    void threshold(cv::Mat &thresholdedBird, cv::Mat &thresholdedWorld) const;
    void mark(cv::Point loc, cv::Scalar color);
    void circle(cv::Point center, int radius, cv::Scalar color);

    void mouseClick(int x, int y);
    
    int getScreenHeight() const {
        assert(boundariesKnown);
        return m_frameHeight;
    }

    int getScreenWidth() const {
        assert(boundariesKnown);
        return m_frameBottomRight.x - m_frameBottomLeft.x;
    }

    int getGroundLevel() const {
        assert(boundariesKnown);
        return m_frameBottomLeft.y;
    }

    int getRightBoundary() const {
        assert(boundariesKnown);
        return m_frameBottomRight.x;
    }

    const cv::Mat& getCurrentFrame() const {
        return m_currentFrame;
    }

    bool boundariesKnown{false};
    void drawLine(cv::Point a, cv::Point b, cv::Scalar color);

    void serialise(cv::FileStorage& storage) const;
    void deserialise(cv::FileStorage& storage);

private:
    void saveBoundaries() const;
    void loadBoundaries();

    cv::Mat m_currentFrame;
    cv::VideoCapture m_cap{-1};

    cv::Point m_clicks[3];
    int m_currentClick{-1};
    cv::Point m_frameBottomLeft;
    cv::Point m_frameBottomRight;
    int m_frameHeight;
};
