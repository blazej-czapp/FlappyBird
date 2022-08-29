#pragma once

#include <optional>
#include "gap.hpp"
#include "units.hpp"

namespace cv {
    class Mat;
}

class VideoFeed;

/**
 * Given thresholded camera captures, finds the location of the bird and pipes/gaps ahead.
 */
class FeatureDetector {
public:
    FeatureDetector(const cv::Mat& world, const cv::Mat& bird, VideoFeed& disp);
    FeatureDetector(const FeatureDetector&) = delete;
    FeatureDetector(FeatureDetector&&) = delete;

    std::pair<std::optional<Gap>, std::optional<Gap>> findGapsAheadOf(Position pos) const;
    std::optional<Position> findBird() const;

private:
    std::optional<Gap> findFirstGapAheadOf(int x) const;
    std::optional<Gap> getGapAt(int x) const;
    int lookUp(int x, int y, int lookFor) const;
    int lookLeft(int x, int y, int lookFor) const;

    const cv::Mat& m_thresholdedMap;
    const cv::Mat& m_thresholdedBird;
    VideoFeed& m_display;
    const int m_lowSweepY; // sweep low to ensure we hit a pipe, not drive through a gap
    const int m_pipeWidth;
    const int m_gapHeight;
};
