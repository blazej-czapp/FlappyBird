#include "display.hpp"
#include "util.hpp"

#include <stdexcept>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

const std::string VideoFeed::FEED_NAME = "Original";

// for [de]serialisation
const static std::string BOUNDARIES_FILE = "boundaries.txt";
const static std::string BOTTOM_LEFT_KEY = "bottom_left";
const static std::string BOTTOM_RIGHT_KEY = "bottom_right";
const static std::string VIEWPORT_HEIGHT_KEY = "viewport_height";
const static std::string UNIT_LENGTH_KEY = "unit_length";

void mouseCallback(int event, int x, int y, int /*flags*/, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        static_cast<VideoFeed *>(userdata)->mouseClick(x, y);
    }
}

VideoFeed::VideoFeed(VideoSource& source) : m_source(source) {
    cv::namedWindow(FEED_NAME);
    cv::setMouseCallback(FEED_NAME, mouseCallback, this);
    loadBoundaries();
}

VideoFeed::~VideoFeed() {}

void VideoFeed::captureFrame() {
    m_currentFrame = m_source.get().captureFrame().clone();
}

void VideoFeed::show() const {
    cv::imshow(FEED_NAME, m_currentFrame);
#ifdef CALIBRATING_DETECTOR
    cv::imshow("Combined", m_imgCombined);
#endif
}

void VideoFeed::mark(cv::Point loc, cv::Scalar color) {
    cv::line(m_currentFrame, cv::Point(loc.x - 20, loc.y), cv::Point(loc.x + 20, loc.y), color, 2);
    cv::line(m_currentFrame, cv::Point(loc.x, loc.y - 20), cv::Point(loc.x, loc.y + 20), color, 2);
}

void VideoFeed::circle(Position center, Distance radius, cv::Scalar color) {
    cv::circle(m_currentFrame, positionToPixel(center), distanceToPixels(radius), color, 2);
}

void VideoFeed::filledCircle(Position center, Distance radius, cv::Scalar color) {
    cv::circle(m_currentFrame, positionToPixel(center), distanceToPixels(radius), color, cv::FILLED);
}

void VideoFeed::mouseClick(int x, int y) {
    m_boundariesKnown = false;
    // viewport boundaries are for active area, probably not the whole frame (e.g. excluding stuff around the tablet)
    switch (m_currentClick) {
        case 0:
            m_frameBottomLeft = cv::Point(x, y);
            mark(m_frameBottomLeft, CV_RED);
            std::cout << "viewport bottom left: " << m_frameBottomLeft << std::endl;
            break;
        case 1:
            m_frameBottomRight = cv::Point(x, y);
            mark(m_frameBottomRight, CV_RED);
            std::cout << "viewport bottom right: " << m_frameBottomRight << std::endl;
            break;
        case 2:
            m_frameHeight = m_frameBottomLeft.y - y;
            mark(cv::Point(x, y), CV_RED); // anywhere along the top edge is fine
            std::cout << "viewport top: " << m_frameHeight << std::endl;
            break;
        case 3:
            m_refBoxLeft = x;
            mark(cv::Point(x, y), CV_BLUE);
            std::cout << "stats box left: " << m_refBoxLeft << std::endl;
            break;
        case 4:
            m_unitLength = x - m_refBoxLeft;
            mark(cv::Point(x, y), CV_BLUE);
            std::cout << "stats box right: " << x << std::endl;
            break;
    }

    m_currentClick = (m_currentClick + 1) % 5;

    if (m_currentClick == 0) {
        std::cout << "Boundaries successfully set" << std::endl;
        m_boundariesKnown = true;
        saveBoundaries();
    }
}

void VideoFeed::saveBoundaries() const {
    cv::FileStorage fs(BOUNDARIES_FILE, cv::FileStorage::WRITE);
    serialise(fs);
    fs.release();
}

void VideoFeed::loadBoundaries() {
    cv::FileStorage fs(BOUNDARIES_FILE, cv::FileStorage::READ);
    if (fs.isOpened()) {
        deserialise(fs);
    }
    fs.release();
}

void VideoFeed::deserialise(cv::FileStorage& storage) {
    storage[BOTTOM_LEFT_KEY] >> m_frameBottomLeft;
    storage[BOTTOM_RIGHT_KEY] >> m_frameBottomRight;
    storage[VIEWPORT_HEIGHT_KEY] >> m_frameHeight;
    storage[UNIT_LENGTH_KEY] >> m_unitLength;
    m_boundariesKnown = true;
}

void VideoFeed::serialise(cv::FileStorage& storage) const {
    assert(boundariesKnown());
    storage << BOTTOM_LEFT_KEY << m_frameBottomLeft << BOTTOM_RIGHT_KEY << m_frameBottomRight
            << VIEWPORT_HEIGHT_KEY << m_frameHeight << UNIT_LENGTH_KEY << m_unitLength;
}
