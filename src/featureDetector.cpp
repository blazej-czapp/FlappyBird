#include "featureDetector.hpp"
#include "display.hpp"
#include "util.hpp"

#include "opencv2/imgproc/imgproc.hpp"

#include <assert.h>
#include <iostream>

const int NOISE_BUFFER = 16;
const int SEARCH_WINDOW_SIZE = 40;
constexpr Distance PIPE_WIDTH{0.237f};
constexpr Distance GAP_HEIGHT{0.460f};

FeatureDetector::FeatureDetector(const cv::Mat &map, const cv::Mat &bird, Display &disp) :
        m_thresholdedMap{map}, m_thresholdedBird{bird}, m_display{disp}, m_lowSweepY{disp.getGroundLevel() - 10},
        m_pipeWidth(disp.distanceToPixels(PIPE_WIDTH)), m_pipeSpacing(m_pipeWidth * 1.5f),
        m_gapHeight(disp.distanceToPixels(GAP_HEIGHT)) {}

int FeatureDetector::lookUp(int x, int y, int lookFor) const {
    for (int row = y; row > 0; --row) {
        if (m_thresholdedMap.ptr<uchar>(row)[x] == lookFor) {
            return row;
        }
    }

    return -1;
}

int FeatureDetector::lookLeft(int x, int y) const {
    for (int i = x; i > x - m_pipeWidth * 2; --i) {
        // assuming no noise inside a pipe
        if (m_thresholdedMap.ptr<uchar>(y)[i] == 0) {
            return i;
        }
    }

    return -1;
}

std::optional<Gap> FeatureDetector::getGapAt(int x) const {
    WARN_UNLESS(m_thresholdedMap.ptr<uchar>(m_lowSweepY)[x] == 255, "looking for a gap at a non-white pixel");
    const int gapY = lookUp(x, m_lowSweepY, 0); // find the bottom of the gap above
    const int gapLeftX = lookLeft(x, m_lowSweepY);

    if (gapY == -1 || gapLeftX == -1) {
        return {};
    }

    const int upperLeftY = gapY - m_gapHeight/*lookUp(x, gapY - m_gapHeight, 255)*/;

    if (upperLeftY == -1) {
        return {};
    }

    Gap gap;
    gap.lowerLeft = Position{m_display.pixelXToPosition(gapLeftX), m_display.pixelYToPosition(gapY)};
    gap.upperLeft = Position{m_display.pixelXToPosition(gapLeftX), m_display.pixelYToPosition(gapY - m_gapHeight)};/*lookUp(x, gapY - m_gapHeight, 255)*/;

    // found the leftmost corners of the obstacle in x, find the rightmost ones
    bool foundRightCorner = false;
    for (int exitX = x + m_pipeWidth * 0.5; exitX < x + m_pipeWidth * 1.5; exitX++) {
        if (m_thresholdedMap.ptr<uchar>(m_lowSweepY)[exitX] == 0) { // are we out of the pipe yet?
            gap.lowerRight = Position{m_display.pixelXToPosition(exitX), gap.lowerLeft.y};
            gap.upperRight = Position{m_display.pixelXToPosition(exitX), gap.upperLeft.y};
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
        Distance detectedWidth = gap.lowerRight.x - gap.lowerLeft.x;
        Distance slack = m_display.pixelsToDistance(m_pipeWidth) - detectedWidth;
        gap.lowerLeft.x -= slack / 2;
        gap.lowerRight.x += slack / 2;
        gap.upperLeft.x -= slack / 2;
        gap.upperRight.x += slack / 2;
    }

    return std::move(gap);
}

// TODO find the far gap sooner, right now it has to be visible in full before it's identified
std::optional<Gap> FeatureDetector::findFirstGapAheadOf(int x) const {
    assert(m_display.boundariesKnown());
    const uchar *row = m_thresholdedMap.ptr<uchar>(m_lowSweepY);
    for (int searchX = x; searchX < m_display.getRightBoundary() -
                                    40; searchX += SEARCH_WINDOW_SIZE) { // -40 for the screen boundary noise (plus it doesn't matter anyway)
        if (searchX + SEARCH_WINDOW_SIZE >= m_display.getRightBoundary()) {
            return {};
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

        return getGapAt(maxWhiteIndex - maxWhiteCount / 2);
    }
    return {};
}

std::pair<std::optional<Gap>, std::optional<Gap>> FeatureDetector::findGapsAheadOf(Position pos) const {
    int x = m_display.positionToPixel(pos).x;
    std::optional<Gap> leftGap = findFirstGapAheadOf(x);
    bool inGap = false;
    if (leftGap.has_value()) {
        if (m_display.positionToPixel(leftGap->lowerLeft).x - x < NOISE_BUFFER) {
            inGap = true;
        }
    }

    if (inGap) {
        // if we're in a gap, then we can see the next pipe ahead
        std::optional<Gap> rightGap = findFirstGapAheadOf(x + m_pipeSpacing);
        //assert(found);

        return { std::move(leftGap), std::move(rightGap) };
    }

    // if we're between pipes or in front of the very first one we can only see one gap ahead
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
        return {m_display.pixelToPosition(cv::Point(posX, posY))};
    }

    return {};
}
