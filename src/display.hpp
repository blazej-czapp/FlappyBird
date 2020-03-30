#pragma once

#include <functional>

#include "opencv2/videoio.hpp"

#include "gap.hpp"
#include "units.hpp"
#include "VideoSource.hpp"

class Display {
    static const std::string FEED_NAME;

public:
    Display(VideoSource& source);
    virtual ~Display();

    void captureFrame();
    void show() const;

    /**
     *
     * @param[in] frame
     * @param[out] birdPos
     * @param[out] thresholdedWorld - thresholded, black&white image of just the obstacles
     */
    void threshold(cv::Mat &thresholdedBird, cv::Mat &thresholdedWorld) const;
    void mark(cv::Point loc, cv::Scalar color);
    void circle(Position center, Distance radius, cv::Scalar color);

    void mouseClick(int x, int y);
    
    int getGroundLevel() const {
        assert(boundariesKnown());
        return m_frameBottomLeft.y;
    }

    int getRightBoundary() const {
        assert(boundariesKnown());
        return m_frameBottomRight.x;
    }

    const cv::Mat& getCurrentFrame() const {
        return m_currentFrame;
    }

    Distance pixelYToPosition(int y) const {
        assert(boundariesKnown());
        return {static_cast<float>(y - (m_frameBottomLeft.y - m_frameHeight)) / m_unitLength};
    }

    Distance pixelXToPosition(int x) const {
        assert(boundariesKnown());
        return {static_cast<float>(x - m_frameBottomLeft.x) / m_unitLength};
    }

    Position pixelToPosition(cv::Point p) const {
        assert(boundariesKnown());
        return Position { pixelXToPosition(p.x), pixelYToPosition(p.y) };
    }

    cv::Point positionToPixel(const Position& pos) {
        assert(boundariesKnown());
        return {static_cast<int>(m_unitLength * pos.x.val + m_frameBottomLeft.x),
                static_cast<int>(m_frameBottomLeft.y - m_frameHeight + m_unitLength * pos.y.val)};
    }

    int distanceToPixels(Distance dist) const {
        assert(boundariesKnown());
        return dist.val * m_unitLength;
    }

    Distance pixelsToDistance(int dist) const {
        assert(boundariesKnown());
        return Distance{static_cast<float>(dist) / m_unitLength};
    }

    bool boundariesKnown() const {
        return m_boundariesKnown;
    };

    void serialise(cv::FileStorage& storage) const;
    void deserialise(cv::FileStorage& storage);

private:
    void saveBoundaries() const;
    void loadBoundaries();

    cv::Mat m_currentFrame;
    const std::reference_wrapper<VideoSource> m_source;

    bool m_boundariesKnown{false};
    int m_currentClick{0};
    cv::Point m_frameBottomLeft;
    cv::Point m_frameBottomRight;
    int m_refBoxLeft; // temp for saving the left edge of the reference unit box (the stats box) while setting boundaries
    int m_unitLength;
    int m_frameHeight;
};
