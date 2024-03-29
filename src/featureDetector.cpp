#include "featureDetector.hpp"
#include "display.hpp"
#include "util.hpp"
#include "constants.hpp"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <assert.h>
#include <iostream>

static constexpr Distance PIPE_SPACING{0.45}; // rough distance between adjacent pipe edges
constexpr int SEARCH_WINDOW_SIZE = 40;
constexpr Distance PIPE_WIDTH{0.251f};
constexpr Distance GAP_HEIGHT{0.487f};
constexpr int WHITE = 255;
constexpr int BLACK = 0;

// HSV filters to capture the bird and the pipes
// int BIRD_LOW_H = 121;
// int BIRD_HIGH_H = 180;
int BIRD_LOW_H = 0;
int BIRD_HIGH_H = 4;

int BIRD_LOW_S = 210;
int BIRD_HIGH_S = 255;

int BIRD_LOW_V = 110;
int BIRD_HIGH_V = 255;

int BEAK_LOW_H = 15;
int BEAK_HIGH_H = 30;

int BEAK_LOW_S = 200;
int BEAK_HIGH_S = 255;

int BEAK_LOW_V = 90;
int BEAK_HIGH_V = 255;

// int PIPES_LOW_H = 7;
// int PIPES_HIGH_H = 88;
int PIPES_LOW_H = 31;
int PIPES_HIGH_H = 50;

// int PIPES_LOW_S = 9;
// int PIPES_HIGH_S = 255;
int PIPES_LOW_S = 47;
int PIPES_HIGH_S = 255;

int PIPES_LOW_V = 43;
int PIPES_HIGH_V = 255;

int MORPHOLOGICAL_OPENING_THRESHOLD = 3;
int MORPHOLOGICAL_CLOSING_THRESHOLD = 13; // 40 // high values slow things down

FeatureDetector::FeatureDetector(VideoFeed &disp) :
        m_display{disp}, m_lowSweepY{disp.getGroundLevel() - 10},
        m_pipeWidth(disp.distanceToPixels(PIPE_WIDTH)),
        m_gapHeight(disp.distanceToPixels(GAP_HEIGHT)) {
#ifdef CALIBRATING_DETECTOR
    cv::namedWindow("Pipe Control", cv::WINDOW_AUTOSIZE); //create a window called "Control"

    // //Create trackbars in "Control" window
    cv::createTrackbar("LowH", "Pipe Control", &PIPES_LOW_H, 180); //Hue (0 - 180)
    cv::createTrackbar("HighH", "Pipe Control", &PIPES_HIGH_H, 180);

    cv::createTrackbar("LowS", "Pipe Control", &PIPES_LOW_S, 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS", "Pipe Control", &PIPES_HIGH_S, 255);

    cv::createTrackbar("LowV", "Pipe Control", &PIPES_LOW_V, 255); //Value (0 - 255)
    cv::createTrackbar("HighV", "Pipe Control", &PIPES_HIGH_V, 255);

    cv::namedWindow("Bird Control", cv::WINDOW_AUTOSIZE); // create a window called "Control"

    //Create trackbars in "Control" window
    cv::createTrackbar("LowH (bird)", "Bird Control", &BIRD_LOW_H, 180); // Hue (0 - 180)
    cv::createTrackbar("HighH (bird)", "Bird Control", &BIRD_HIGH_H, 180);

    cv::createTrackbar("LowS (bird)", "Bird Control", &BIRD_LOW_S, 255); // Saturation (0 - 255)
    cv::createTrackbar("HighS (bird)", "Bird Control", &BIRD_HIGH_S, 255);

    cv::createTrackbar("LowV (bird)", "Bird Control", &BIRD_LOW_V, 255); // Value (0 - 255)
    cv::createTrackbar("HighV (bird)", "Bird Control", &BIRD_HIGH_V, 255);

    cv::createTrackbar("LowH (beak)", "Bird Control", &BEAK_LOW_H, 180); // Hue (0 - 180)
    cv::createTrackbar("HighH (beak)", "Bird Control", &BEAK_HIGH_H, 180);

    cv::createTrackbar("LowS (beak)", "Bird Control", &BEAK_LOW_S, 255); // Saturation (0 - 255)
    cv::createTrackbar("HighS (beak)", "Bird Control", &BEAK_HIGH_S, 255);

    cv::createTrackbar("LowV (beak)", "Bird Control", &BEAK_LOW_V, 255); // Value (0 - 255)
    cv::createTrackbar("HighV (beak)", "Bird Control", &BEAK_HIGH_V, 255);

    // //Create trackbars in "Control" window
    cv::createTrackbar("Opening", "Bird Control", &MORPHOLOGICAL_OPENING_THRESHOLD, 40);
    cv::createTrackbar("Closing", "Bird Control", &MORPHOLOGICAL_CLOSING_THRESHOLD, 80);
#endif // CALIBRATING_DETECTOR
}

// keep looking up this many pixels after finding what we're looking for to bridge gaps
// within pipes (especially the vertical black segments around the crown of a pipe)
static const int CONFIDENCE_BUFFER = 20;
int FeatureDetector::lookUp(int x, int y, int lookFor) const {
    int confidence = 0;
    for (int row = y; row > 0; --row) {
        if (m_thresholdedWorld.ptr<uchar>(row)[x] == lookFor) {
            if (confidence == CONFIDENCE_BUFFER) {
                return row + confidence;
            } else {
                ++confidence;
            }
        } else {
            confidence = 0;
        }
    }

    return -1;
}

int FeatureDetector::lookLeft(int x, int y, int lookFor) const {
    // 1.1 just in case we're exactly at the right edge
    for (int i = x; i > x - m_pipeWidth * 1.1; --i) {
        // assuming no noise inside a pipe
        if (m_thresholdedWorld.ptr<uchar>(y)[i] == lookFor) {
            return i;
        }
    }

    return -1;
}

std::optional<Gap> FeatureDetector::getGapAt(int x) const {
    WARN_UNLESS(m_thresholdedWorld.ptr<uchar>(m_lowSweepY)[x] == WHITE, "looking for a gap at a non-white pixel");
    const int gapY = lookUp(x, m_lowSweepY, BLACK); // find the bottom of the gap above
    // look a little below the bottom of the gap to miss the notch around the crown
    const int gapLeftX = lookLeft(x, gapY + 4, BLACK);

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

std::optional<Gap> FeatureDetector::findFirstGapAheadOf(int x) const {
    assert(m_display.boundariesKnown());
    const uchar *row = m_thresholdedWorld.ptr<uchar>(m_lowSweepY);
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

        if (maxWhiteCount < static_cast<float>(SEARCH_WINDOW_SIZE) / 4) {
            continue;
        }

        return getGapAt(maxWhiteIndex - maxWhiteCount / 2);
    }
    return {};
}

std::pair<std::optional<Gap>, std::optional<Gap>> FeatureDetector::findGapsAheadOf(Position pos) const {
    int x = m_display.positionToPixel(pos).x - m_display.distanceToPixels(BIRD_RADIUS);
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
        // Position pos = m_display.pixelToPosition(cv::Point(posX, posY));
        // TODO x position is constant, but we should set it manually when defining
        //      viewport, not hardcoding
        Position pos{BIRD_X_COORDINATE, m_display.pixelYToPosition(posY)};
        pos.y.val -= 0.01;

        return pos;
    }

    return {};
}

void openClose(const cv::Mat& imgHsvIn, cv::Mat& imgOut, int lowH, int highH, int lowS, int highS, int lowV, int highV) {
    cv::inRange(imgHsvIn, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), imgOut); //Threshold the image

    // this stuff is nice, but it's slow - we can get good results with some manual
    // ray casting (at least with screen grab, will see how it goes with webcam)
    //morphological opening (removes small objects from the foreground)
    // cv::erode(imgOut, imgOut, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPHOLOGICAL_OPENING_THRESHOLD,
    //                                                                             MORPHOLOGICAL_OPENING_THRESHOLD)));
    // cv::dilate(imgOut, imgOut, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPHOLOGICAL_OPENING_THRESHOLD,
    //                                                                              MORPHOLOGICAL_OPENING_THRESHOLD)));

    //morphological closing (removes small holes from the foreground)
    // cv::dilate(imgOut, imgOut, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPHOLOGICAL_CLOSING_THRESHOLD,
    //                                                                              MORPHOLOGICAL_CLOSING_THRESHOLD)));
    // cv::erode(imgOut, imgOut, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPHOLOGICAL_CLOSING_THRESHOLD,
    //                                                                             MORPHOLOGICAL_CLOSING_THRESHOLD)));
}

static cv::Mat imgHSV;
void FeatureDetector::process(const cv::Mat& frame) {

    cv::cvtColor(frame, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

#ifdef CALIBRATING_DETECTOR
    // process the entire frame so that we can add it to m_imgCombined
    const cv::Rect birdColumn(0, 0, imgHSV.cols, imgHSV.rows);
#else
    // in 'production' we only need to process the column we know the bird occupies
    const cv::Rect birdColumn(imgHSV.cols*0.2, 0, imgHSV.cols*0.4, imgHSV.rows);
#endif

    // Crop the full image to that image contained by the rectangle birdColumn
    // Note that this doesn't copy the data
    cv::Mat croppedImage = imgHSV(birdColumn);

    openClose(croppedImage, m_thresholdedBird, BIRD_LOW_H, BIRD_HIGH_H, BIRD_LOW_S, BIRD_HIGH_S, BIRD_LOW_V, BIRD_HIGH_V);
    openClose(croppedImage, m_thresholdedBeak, BEAK_LOW_H, BEAK_HIGH_H, BEAK_LOW_S, BEAK_HIGH_S, BEAK_LOW_V,
              BEAK_HIGH_V);
    openClose(imgHSV, m_thresholdedWorld, PIPES_LOW_H, PIPES_HIGH_H, PIPES_LOW_S, PIPES_HIGH_S, PIPES_LOW_V, PIPES_HIGH_V);

    m_thresholdedBird += m_thresholdedBeak;

#ifdef CALIBRATING_DETECTOR
    m_imgCombined = m_thresholdedWorld + m_thresholdedBird + m_thresholdedBeak;
    cv::imshow("Combined", m_imgCombined);
#endif
}
