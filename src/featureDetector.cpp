#include "featureDetector.hpp"
#include "display.hpp"
#include "util.hpp"

#include "opencv2/imgproc/imgproc.hpp"

#include <assert.h>
#include <iostream>

static constexpr Distance PIPE_SPACING{0.45}; // rough distance between adjacent pipe edges
constexpr int SEARCH_WINDOW_SIZE = 40;
constexpr Distance PIPE_WIDTH{0.237f};
// constexpr Distance GAP_HEIGHT{0.460f};
constexpr Distance GAP_HEIGHT{0.480f};
constexpr int WHITE = 255;
constexpr int BLACK = 0;

FeatureDetector::FeatureDetector(const cv::Mat &map, const cv::Mat &bird, VideoFeed &disp) :
        m_thresholdedMap{map}, m_thresholdedBird{bird}, m_display{disp}, m_lowSweepY{disp.getGroundLevel() - 10},
        m_pipeWidth(disp.distanceToPixels(PIPE_WIDTH)),
        m_gapHeight(disp.distanceToPixels(GAP_HEIGHT)) {}

int FeatureDetector::lookUp(int x, int y, int lookFor) const {
    for (int row = y; row > 0; --row) {
        if (m_thresholdedMap.ptr<uchar>(row)[x] == lookFor) {
            return row;
        }
    }

    return -1;
}

int FeatureDetector::lookLeft(int x, int y, int lookFor) const {
    // 1.1 just in case we're exactly at the right edge
    for (int i = x; i > x - m_pipeWidth * 1.1; --i) {
        // assuming no noise inside a pipe
        if (m_thresholdedMap.ptr<uchar>(y)[i] == lookFor) {
            return i;
        }
    }

    return -1;
}

std::optional<Gap> FeatureDetector::getGapAt(int x) const {
    WARN_UNLESS(m_thresholdedMap.ptr<uchar>(m_lowSweepY)[x] == WHITE, "looking for a gap at a non-white pixel");
    const int gapY = lookUp(x, m_lowSweepY, BLACK); // find the bottom of the gap above
    const int gapLeftX = lookLeft(x, m_lowSweepY, BLACK);

    if (gapY == -1 || gapLeftX == -1) {
        return {};
    }

    Gap gap;
    gap.lowerLeft = Position{m_display.pixelXToPosition(gapLeftX), m_display.pixelYToPosition(gapY)};
    gap.upperLeft = Position{m_display.pixelXToPosition(gapLeftX), m_display.pixelYToPosition(gapY - m_gapHeight)};
    gap.lowerRight = Position{m_display.pixelXToPosition(gapLeftX + m_pipeWidth), m_display.pixelYToPosition(gapY)};
    gap.upperRight = Position{m_display.pixelXToPosition(gapLeftX + m_pipeWidth), m_display.pixelYToPosition(gapY - m_gapHeight)};

    // The boundaries of the pipe have different colour than the middle (roughly the same amount on each side)
    // and they're not detected as well but pipes are always the same width, so tweak the detected corners
    // symmetrically (by adding the missing width, which we know by measuring it manually).
    Distance detectedWidth = gap.lowerRight.x - gap.lowerLeft.x;
    Distance slack = m_display.pixelsToDistance(m_pipeWidth) - detectedWidth;
    gap.lowerLeft.x -= slack / 2;
    gap.lowerRight.x += slack / 2;
    gap.upperLeft.x -= slack / 2;
    gap.upperRight.x += slack / 2;

    const Distance offset{-0.015f};
    gap.lowerLeft.x += offset;
    gap.lowerRight.x += offset;
    gap.upperLeft.x += offset;
    gap.upperRight.x += offset;

    const Distance vertOffset{-0.01f};
    gap.lowerLeft.y += vertOffset;
    gap.lowerRight.y += vertOffset;

    return std::move(gap);
}

// TODO find the far gap sooner, right now it has to be visible in full before it's identified
std::optional<Gap> FeatureDetector::findFirstGapAheadOf(int x) const {
    assert(m_display.boundariesKnown());
    const uchar *row = m_thresholdedMap.ptr<uchar>(m_lowSweepY);
    int rightBoundary = m_display.getRightBoundary();
    for (int searchX = x; searchX < rightBoundary; searchX += SEARCH_WINDOW_SIZE) {
        // Check the SEARCH_WINDOW_SIZE pixels ahead if we have a white block.
        // This works with te assumption that a row is composed sequences of solid black and solid white, with only
        // sporadic noise outside of the pipe. In the end, maxWhiteCount should be roughly equal to the length of the
        // white block within the current window and maxWhiteIndex will point to the end of that block - we'll then cast
        // a ray up from the mid point of the block to find the gap;
        unsigned maxWhiteCount = 0;
        unsigned maxWhiteIndex = 0;
        unsigned currentWhiteCount = 0;

        // We may not have a full search window if looking at a far pipe just emerging from the edge of the screen.
        for (unsigned i = searchX; i < std::min(rightBoundary, searchX + SEARCH_WINDOW_SIZE); ++i) {
            if (row[i] == WHITE) {
                ++currentWhiteCount;
                if (currentWhiteCount > maxWhiteCount) {
                    maxWhiteCount = currentWhiteCount;
                    maxWhiteIndex = i;
                }
            } else {
                currentWhiteCount = 0;
            }
        }

        if (maxWhiteCount < static_cast<float>(SEARCH_WINDOW_SIZE) / 8) {
            continue;
        }

        return getGapAt(maxWhiteIndex - maxWhiteCount / 2);
    }
    return {};
}

std::pair<std::optional<Gap>, std::optional<Gap>> FeatureDetector::findGapsAheadOf(Position pos) const {
    int x = m_display.positionToPixel(pos).x;
    std::optional<Gap> leftGap = findFirstGapAheadOf(x);
    if (leftGap.has_value()) {
        // we may be able to see the next gap as well (or part thereof)
        std::optional<Gap> rightGap = findFirstGapAheadOf(
                m_display.coordinateXToPixel(leftGap->lowerRight.x + PIPE_SPACING));

        return { std::move(leftGap), std::move(rightGap) };
    }

    return { std::move(leftGap), {} };
}

std::optional<Position> FeatureDetector::findBird() const {
    //Calculate the moments of the thresholded image
    cv::Moments oMoments = cv::moments(m_thresholdedBird);

    double dM01 = oMoments.m01;
    double dM10 = oMoments.m10;
    double dArea = oMoments.m00;

    // if the area <= 10000 - assume no object in the image (presumably due to noise)
    if (dArea > 10000) {
        //calculate the m_position of the ball
        int posX = dM10 / dArea;
        int posY = dM01 / dArea;
        Position pos = m_display.pixelToPosition(cv::Point(posX, posY));
        pos.x.val += 0.01;

        return pos;
    }

    return {};
}
