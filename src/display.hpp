#pragma once

#include <functional>

#include "opencv2/videoio.hpp"

#include "gap.hpp"
#include "units.hpp"
#include "VideoSource.hpp"

class VideoFeed { //TODO find better name (or use namespace, Display conflicts with X11)
    static const std::string FEED_NAME;

public:
    VideoFeed(VideoSource& source);
    virtual ~VideoFeed();

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
    void filledCircle(Position center, Distance radius, cv::Scalar color);

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

    Coordinate pixelXToPosition(int x) const {
        assert(boundariesKnown());
        return {static_cast<float>(x - m_frameBottomLeft.x) / m_unitLength};
    }

    Coordinate pixelYToPosition(int y) const {
        assert(boundariesKnown());
        return {static_cast<float>(y - (m_frameBottomLeft.y - m_frameHeight)) / m_unitLength};
    }

    Position pixelToPosition(cv::Point p) const {
        assert(boundariesKnown());
        return Position { pixelXToPosition(p.x), pixelYToPosition(p.y) };
    }

    int coordinateXToPixel(const Coordinate& coord) {
        return static_cast<int>(m_unitLength * coord.val + m_frameBottomLeft.x);
    }

    int coordinateYToPixel(const Coordinate& coord) {
        return static_cast<int>(m_frameBottomLeft.y - m_frameHeight + m_unitLength * coord.val);
    }

    cv::Point positionToPixel(const Position& pos) {
        assert(boundariesKnown());
        return {coordinateXToPixel(pos.x), coordinateYToPixel(pos.y)};
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
