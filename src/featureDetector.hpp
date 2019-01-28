#pragma once

#include "gap.hpp"

namespace cv {
    class Mat;
}

class Display;

/**
 * Given thresholded camera captures, finds the location of the bird and pipes/gaps ahead.
 */
class FeatureDetector {
public:
    FeatureDetector(const cv::Mat& world, const cv::Mat& bird, Display& cam);
    FeatureDetector(const FeatureDetector&) = delete;
    FeatureDetector(FeatureDetector&&) = delete;

    int findGapsAheadOf(int x, Gap& left, Gap& right) const;
    cv::Point findBird() const;

private:
    bool findFirstGapAheadOf(int x, Gap& gap) const;
    bool getGapAt(int x, Gap& gap) const;
    int lookUp(int x, int y, int lookFor) const;
    int lookLeft(int x, int y) const;

    const cv::Mat& m_thresholdedMap;
    const cv::Mat& m_thresholdedBird;
    Display& m_cam;
    const int m_lowSweepY; // sweep low to ensure we hit a pipe, not drive through a gap
    const int m_pipeWidth;
    const int m_pipeSpacing;
    const int m_gapHeight;
};
