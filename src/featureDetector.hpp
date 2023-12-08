#pragma once

#include <optional>
#include "gap.hpp"
#include "units.hpp"

#include "opencv2/imgproc/imgproc.hpp"

class VideoFeed;

/**
 * Given thresholded camera captures, finds the location of the bird and pipes/gaps ahead.
 */
class FeatureDetector {
public:
    FeatureDetector(VideoFeed& disp);
    FeatureDetector(const FeatureDetector&) = delete;
    FeatureDetector(FeatureDetector&&) = delete;

    // video frame in BGR format to perform feature detection on
    void process(const cv::Mat& frame);
    std::pair<std::optional<Gap>, std::optional<Gap>> findGapsAheadOf(Position pos) const;
    std::optional<Position> findBird() const;

private:
    std::optional<Gap> findFirstGapAheadOf(int x) const;
    std::optional<Gap> getGapAt(int x) const;
    int lookUp(int x, int y, int lookFor) const;
    int lookLeft(int x, int y, int lookFor) const;

    cv::Mat m_thresholdedBird;
    cv::Mat m_thresholdedBeak;
    cv::Mat m_thresholdedWorld;
#ifdef CALIBRATING_DETECTOR
    cv::Mat m_imgCombined;
#endif
    VideoFeed& m_display;
    const int m_lowSweepY; // sweep low to ensure we hit a pipe, not drive through a gap
    const int m_pipeWidth;
    const int m_gapHeight;
};
