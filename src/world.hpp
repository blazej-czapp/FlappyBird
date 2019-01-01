#pragma once

#include "gap.hpp"

namespace cv {
    class Mat;
}

class Camera;

class World {
public:
    World(const cv::Mat& map, Camera& cam);
    World(const World&) = delete;
    World(World&&) = delete;

    int findGapsAheadOf(int x, Gap& left, Gap& right) const;

private:
    bool findFirstGapAheadOf(int x, Gap& gap) const;
    bool getGapAt(int x, Gap& gap) const;
    int lookUp(int x, int y, int lookFor) const;
    int lookLeft(int x, int y) const;
    void markGap(const Gap& gap) const;

    const cv::Mat& m_map;
    Camera& m_cam;
    const int m_lowSweepY; // sweep low to ensure we hit a pipe, not fly through a gap
    const int m_pipeWidth;
    const int m_pipeSpacing;
    const int m_gapHeight;
};
