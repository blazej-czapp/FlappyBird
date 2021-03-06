#include "display.hpp"
#include "util.hpp"

#include <stdexcept>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

const std::string Display::FEED_NAME = "Original";

// HSV filters to capture the bird and the pipes
int BIRD_LOW_H = 121;
int BIRD_HIGH_H = 180;

int BIRD_LOW_S = 165;
int BIRD_HIGH_S = 255;

int BIRD_LOW_V = 110;
int BIRD_HIGH_V = 255;

int PIPES_LOW_H = 7;
int PIPES_HIGH_H = 88;

int PIPES_LOW_S = 9;
int PIPES_HIGH_S = 255;

int PIPES_LOW_V = 43;
int PIPES_HIGH_V = 255;

int MORPHOLOGICAL_OPENING_THRESHOLD = 3;
int MORPHOLOGICAL_CLOSING_THRESHOLD = 20; // high values slow things down

// for [de]serialisation
const static std::string BOUNDARIES_FILE = "boundaries.txt";
const static std::string BOTTOM_LEFT_KEY = "bottom_left";
const static std::string BOTTOM_RIGHT_KEY = "bottom_right";
const static std::string VIEWPORT_HEIGHT_KEY = "viewport_height";
const static std::string UNIT_LENGTH_KEY = "unit_length";

void mouseCallback(int event, int x, int y, int /*flags*/, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        static_cast<Display *>(userdata)->mouseClick(x, y);
    }
}

Display::Display(VideoSource& source) : m_source(source) {
    cv::namedWindow(FEED_NAME);
    cv::setMouseCallback(FEED_NAME, mouseCallback, this);
    loadBoundaries();
    cv::namedWindow("Pipe Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    // //Create trackbars in "Control" window
    cv::createTrackbar("LowH", "Pipe Control", &PIPES_LOW_H, 180); //Hue (0 - 180)
    cv::createTrackbar("HighH", "Pipe Control", &PIPES_HIGH_H, 180);

    cv::createTrackbar("LowS", "Pipe Control", &PIPES_LOW_S, 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS", "Pipe Control", &PIPES_HIGH_S, 255);

    cv::createTrackbar("LowV", "Pipe Control", &PIPES_LOW_V, 255); //Value (0 - 255)
    cv::createTrackbar("HighV", "Pipe Control", &PIPES_HIGH_V, 255);

    cv::namedWindow("Driver Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    // //Create trackbars in "Control" window
    cv::createTrackbar("LowH", "Driver Control", &BIRD_LOW_H, 180); //Hue (0 - 180)
    cv::createTrackbar("HighH", "Driver Control", &BIRD_HIGH_H, 180);

    cv::createTrackbar("LowS", "Driver Control", &BIRD_LOW_S, 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS", "Driver Control", &BIRD_HIGH_S, 255);

    cv::createTrackbar("LowV", "Driver Control", &BIRD_LOW_V, 255); //Value (0 - 255)
    cv::createTrackbar("HighV", "Driver Control", &BIRD_HIGH_V, 255);

    // //Create trackbars in "Control" window
    cv::createTrackbar("Opening", "Driver Control", &MORPHOLOGICAL_OPENING_THRESHOLD, 40);
    cv::createTrackbar("Closing", "Driver Control", &MORPHOLOGICAL_CLOSING_THRESHOLD, 80);
}

Display::~Display() {}

void openClose(const cv::Mat& imgHsvIn, cv::Mat& imgOut, int lowH, int highH, int lowS, int highS, int lowV, int highV) {
    cv::inRange(imgHsvIn, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), imgOut); //Threshold the image

    //morphological opening (removes small objects from the foreground)
    cv::erode(imgOut, imgOut, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPHOLOGICAL_OPENING_THRESHOLD,
                                                                                MORPHOLOGICAL_OPENING_THRESHOLD)));
    cv::dilate(imgOut, imgOut, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPHOLOGICAL_OPENING_THRESHOLD,
                                                                                 MORPHOLOGICAL_OPENING_THRESHOLD)));

    //morphological closing (removes small holes from the foreground)
    cv::dilate(imgOut, imgOut, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPHOLOGICAL_CLOSING_THRESHOLD,
                                                                                 MORPHOLOGICAL_CLOSING_THRESHOLD)));
    cv::erode(imgOut, imgOut, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPHOLOGICAL_CLOSING_THRESHOLD,
                                                                                MORPHOLOGICAL_CLOSING_THRESHOLD)));
}

void Display::captureFrame() {
    m_currentFrame = m_source.get().captureFrame().clone();
}

void Display::show() const {
    cv::imshow(FEED_NAME, m_currentFrame);
}

void Display::threshold(cv::Mat& thresholdedBird, cv::Mat& thresholdedWorld) const {
    static cv::Mat imgHSV;

    cv::cvtColor(m_currentFrame, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    openClose(imgHSV, thresholdedBird, BIRD_LOW_H, BIRD_HIGH_H, BIRD_LOW_S, BIRD_HIGH_S, BIRD_LOW_V, BIRD_HIGH_V);
    openClose(imgHSV, thresholdedWorld, PIPES_LOW_H, PIPES_HIGH_H, PIPES_LOW_S, PIPES_HIGH_S, PIPES_LOW_V, PIPES_HIGH_V);

    static cv::Mat imgCombined;
    imgCombined = thresholdedWorld + thresholdedBird;
    cv::imshow("Combined", imgCombined); //TODO a bit cheeky to be displaying something in threshold()...
}

void Display::mark(cv::Point loc, cv::Scalar color) {
    cv::line(m_currentFrame, cv::Point(loc.x - 20, loc.y), cv::Point(loc.x + 20, loc.y), color, 2);
    cv::line(m_currentFrame, cv::Point(loc.x, loc.y - 20), cv::Point(loc.x, loc.y + 20), color, 2);
}

void Display::circle(Position center, Distance radius, cv::Scalar color) {
    cv::circle(m_currentFrame, positionToPixel(center), distanceToPixels(radius), color, 2);
}

void Display::mouseClick(int x, int y) {
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
        m_boundariesKnown = true;
        saveBoundaries();
    }
}

void Display::saveBoundaries() const {
    cv::FileStorage fs(BOUNDARIES_FILE, cv::FileStorage::WRITE);
    serialise(fs);
    fs.release();
}

void Display::loadBoundaries() {
    cv::FileStorage fs(BOUNDARIES_FILE, cv::FileStorage::READ);
    if (fs.isOpened()) {
        deserialise(fs);
    }
    fs.release();
}

void Display::deserialise(cv::FileStorage& storage) {
    storage[BOTTOM_LEFT_KEY] >> m_frameBottomLeft;
    storage[BOTTOM_RIGHT_KEY] >> m_frameBottomRight;
    storage[VIEWPORT_HEIGHT_KEY] >> m_frameHeight;
    storage[UNIT_LENGTH_KEY] >> m_unitLength;
    m_boundariesKnown = true;
}

void Display::serialise(cv::FileStorage& storage) const {
    assert(boundariesKnown());
    storage << BOTTOM_LEFT_KEY << m_frameBottomLeft << BOTTOM_RIGHT_KEY << m_frameBottomRight
            << VIEWPORT_HEIGHT_KEY << m_frameHeight << UNIT_LENGTH_KEY << m_unitLength;
}
