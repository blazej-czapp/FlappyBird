#include "world.hpp"
#include "camera.hpp"
#include "util.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <assert.h>
#include <iostream>
#include <algorithm>

const int NOISE_BUFFER = 16;
const int SEARCH_WINDOW_SIZE = 40;
const float PIPE_WIDTH = 0.18f; // as proportion of screen width
const float GAP_HEIGHT = 0.35f; // as proportion of screen width

World::World(const cv::Mat& map, Camera& cam) :
    m_map(map), m_cam(cam), m_lowSweepY(cam.getGroundLevel() - 10),
    m_pipeWidth(cam.getScreenWidth() * PIPE_WIDTH), m_pipeSpacing(m_pipeWidth * 1.5f),
    m_gapHeight(cam.getScreenWidth() * GAP_HEIGHT) {}

int World::lookUp(int x, int y, int lookFor) const {
    for (int row = y; row > 0; --row) {
        if (m_map.ptr<uchar>(row)[x] == lookFor) {
            return row;
        }
    }

    return -1;
}

int World::lookLeft(int x, int y) const {
    for (int i = x; i > x - m_pipeWidth*2; --i) {
        // assuming no noise inside a pipe
        if (m_map.ptr<uchar>(y)[i] == 0) {
            return i;
        }
    }

    return -1;
}

bool World::getGapAt(int x, Gap& gap) const {
    WARN_UNLESS(m_map.ptr<uchar>(m_lowSweepY)[x] == 255, "looking for a gap at a non-white pixel");
    int gapY = lookUp(x, m_lowSweepY, 0); // find the bottom of the gap above
    int gapLeftX = lookLeft(x, m_lowSweepY);

    if (gapY == -1 || gapLeftX == -1) {
        return false;
    }
    gap.lowerLeft = cv::Point(gapLeftX, gapY);
    gap.upperLeft = cv::Point(gapLeftX, gapY - m_gapHeight/*lookUp(x, gapY - m_gapHeight, 255)*/);

    if (gap.upperLeft.y == -1) {
        return false;
    }

    // found the leftmost corners of the obstacle in x, find the rightmost ones
    bool foundRightCorner = false;
    for (int exitX = x + m_pipeWidth * 0.5; exitX < x + m_pipeWidth * 1.5; exitX++) {
        if (m_map.ptr<uchar>(m_lowSweepY)[exitX] == 0) { // are we out of the pipe yet?
            gap.lowerRight = cv::Point(exitX, gap.lowerLeft.y);
            gap.upperRight = cv::Point(exitX, gap.upperLeft.y);
            foundRightCorner = true;
            break;
        }
    }

    // it's possible for the right side to be past the screen edge (for the far obstacle), this is fine
    if (!foundRightCorner) {
        gap.lowerRight = gap.lowerLeft;
        gap.upperRight = gap.upperLeft;
    } else {
        // the boundaries of the pipe have different colour than the middle (roughly the same amount on each side)
        // and they're not detected as well but pipes are always the same width, so tweak the detected corners
        // symmetrically
        int detectedWidth = gap.lowerRight.x - gap.lowerLeft.x;
        int slack = m_pipeWidth - detectedWidth;
        gap.lowerLeft.x -= slack / 2;
        gap.lowerRight.x += slack / 2;
        gap.upperLeft.x -= slack / 2;
        gap.upperRight.x += slack / 2;
    }

    return true;
}

bool World::findFirstGapAheadOf(int x, Gap& gap) const {
    assert(m_cam.boundariesKnown);
    m_cam.drawLine(cv::Point(x, m_lowSweepY), cv::Point(x+NOISE_BUFFER, m_lowSweepY), cv::Scalar(255, 255, 0));
    const uchar* row = m_map.ptr<uchar>(m_lowSweepY);
    for (unsigned searchX = x; searchX < m_cam.getRightBoundary() - 40; searchX += SEARCH_WINDOW_SIZE) { // -40 for the screen boundary noise (plus it doesn't matter anyway)
        if (searchX + SEARCH_WINDOW_SIZE >= m_cam.getRightBoundary()) {
            return false;
        }

        // check the SEARCH_WINDOW_SIZE pixels ahead if it contains a white block
        // this works with te assumption that a row is composed sequences of solid black and solid white with only
        // sporadic noise; in the end, maxWhiteCount should be roughly equal to the length of the white block within
        // the current window and maxWhiteIndex will point to the end of that block - we'll then cast a ray up from
        // the mid point of that block to find the gap;
        // we'll also assume that no block is smaller than SEARCH_WINDOW_SIZE so subtracting maxWhiteCount from
        // maxWhiteIndex should give us roughly the middle point of the white block
        unsigned maxWhiteCount = 0;
        unsigned maxWhiteIndex = 0;
        unsigned currentWhiteCount = 0;
        for (unsigned i = searchX; i < searchX + SEARCH_WINDOW_SIZE; ++i) {
            if (row[i] == 255) {
                ++currentWhiteCount;
                if (currentWhiteCount > maxWhiteCount) {
                    maxWhiteCount = currentWhiteCount;
                    maxWhiteIndex = i;
                }
            }
        }

        if (maxWhiteCount < SEARCH_WINDOW_SIZE / 2) {
            continue;
        }

        return getGapAt(maxWhiteIndex - maxWhiteCount / 2, gap);
    }
    return false;
}

void World::markGap(const Gap& gap) const {
    m_cam.mark(gap.lowerLeft, cv::Scalar(255, 0, 0));
    m_cam.mark(gap.lowerRight, cv::Scalar(255, 0, 0));
    m_cam.mark(gap.upperLeft, cv::Scalar(0, 0, 255));
    m_cam.mark(gap.upperRight, cv::Scalar(0, 0, 255));
}

int World::findGapsAheadOf(int x, Gap& left, Gap& right) const {
    bool inGap = false;
    bool foundGap = false;
    if (findFirstGapAheadOf(x, left)) {
        foundGap = true;
        markGap(left);
        if (left.lowerLeft.x - x < NOISE_BUFFER) {
            inGap = true;
        }
    }

    if (inGap) {
        // if we're in a gap, then we can see the next pipe ahead
        bool found = findFirstGapAheadOf(x + m_pipeSpacing, right);
        //assert(found);
        markGap(right);

        return 2;
    } else if (foundGap) {
        // we're in-between pipes (or in front of the very first one) and we can only see one ahead so return
        return 1;
    } else {
        return 0;
    }
}
