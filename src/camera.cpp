#include "camera.hpp"
#include "bird.hpp"
#include "util.hpp"

#include <iostream>
#include <fstream>
#include <unistd.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

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
int MORPHOLOGICAL_CLOSING_THRESHOLD = 30; // high values slow things down

void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        static_cast<Camera *>(userdata)->mouseClick(x, y);
    }
}

Camera::Camera() : m_showFeed(true), m_feedName("Original"), m_cap(-1), m_currentClick(-1), boundariesKnown(false) {
    if (!m_cap.isOpened()) { // if not successful, exit program
        std::cout << "Cannot open the web cam" << std::endl;
        throw "Cannot open the web cam";
    }
    cv::Mat imgTmp;
    m_cap.read(imgTmp);

    cv::imshow(m_feedName, imgTmp);
    cv::setMouseCallback(m_feedName, mouseCallback, this);
    loadBoundaries();
    cv::namedWindow("Pipe Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    // //Create trackbars in "Control" window
    cv::createTrackbar("LowH", "Pipe Control", &PIPES_LOW_H, 180); //Hue (0 - 180)
    cv::createTrackbar("HighH", "Pipe Control", &PIPES_HIGH_H, 180);

    cv::createTrackbar("LowS", "Pipe Control", &PIPES_LOW_S, 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS", "Pipe Control", &PIPES_HIGH_S, 255);

    cv::createTrackbar("LowV", "Pipe Control", &PIPES_LOW_V, 255); //Value (0 - 255)
    cv::createTrackbar("HighV", "Pipe Control", &PIPES_HIGH_V, 255);

    cv::namedWindow("Bird Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    // //Create trackbars in "Control" window
    cv::createTrackbar("LowH", "Bird Control", &BIRD_LOW_H, 180); //Hue (0 - 180)
    cv::createTrackbar("HighH", "Bird Control", &BIRD_HIGH_H, 180);

    cv::createTrackbar("LowS", "Bird Control", &BIRD_LOW_S, 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS", "Bird Control", &BIRD_HIGH_S, 255);

    cv::createTrackbar("LowV", "Bird Control", &BIRD_LOW_V, 255); //Value (0 - 255)
    cv::createTrackbar("HighV", "Bird Control", &BIRD_HIGH_V, 255);

    // //Create trackbars in "Control" window
    cv::createTrackbar("Opening", "Bird Control", &MORPHOLOGICAL_OPENING_THRESHOLD, 40);
    cv::createTrackbar("Closing", "Bird Control", &MORPHOLOGICAL_CLOSING_THRESHOLD, 80);
}

Camera::~Camera() {}

cv::Point getBirdPos(const cv::Mat& birdImg) {
    //Calculate the moments of the thresholded image
    cv::Moments oMoments = cv::moments(birdImg);

    double dM01 = oMoments.m01;
    double dM10 = oMoments.m10;
    double dArea = oMoments.m00;

    // if the area <= 10000 - assume no object in the image (presumably due to noise)
    if (dArea > 10000) {
        //calculate the m_position of the ball
        int posX = dM10 / dArea;
        int posY = dM01 / dArea;
        return cv::Point(posX, posY);
    }

    return cv::Point(-1, -1);
}

void threshold(const cv::Mat& imgHsvIn, cv::Mat& imgOut, int lowH, int highH, int lowS, int highS, int lowV, int highV,
               int openingThreshold) {
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

void Camera::capture(cv::Point& birdPos, cv::Mat& world) {
    //Capture a temporary image from the camera
    static cv::Mat imgTmp;
    m_cap.read(imgTmp);

    bool bSuccess = m_cap.read(m_currentFrame); // read a new frame from video
    cv::flip(m_currentFrame, m_currentFrame, -1);
    if (!bSuccess) {
        throw "Cannot read a frame from video stream";
    }

    static cv::Mat imgHSV;

    cv::cvtColor(m_currentFrame, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    static cv::Mat imgThresholdedBird;

    threshold(imgHSV, imgThresholdedBird, BIRD_LOW_H, BIRD_HIGH_H, BIRD_LOW_S, BIRD_HIGH_S, BIRD_LOW_V, BIRD_HIGH_V, MORPHOLOGICAL_OPENING_THRESHOLD);
    threshold(imgHSV, world, PIPES_LOW_H, PIPES_HIGH_H, PIPES_LOW_S, PIPES_HIGH_S, PIPES_LOW_V, PIPES_HIGH_V, MORPHOLOGICAL_OPENING_THRESHOLD);

    birdPos = getBirdPos(imgThresholdedBird);
    circle(birdPos, getScreenWidth() * Bird::RADIUS, CV_BLUE);
    mark(birdPos, cv::Scalar(255, 0, 0));

    if (m_showFeed) {
        static cv::Mat imgCombined;
        imgCombined = world + imgThresholdedBird;
        cv::imshow("Combined", imgCombined);
        cv::imshow(m_feedName, m_currentFrame);
    }
}

void Camera::drawLine(const cv::Point& a, const cv::Point& b, const cv::Scalar& color) {
    cv::line(m_currentFrame, a, b, color, 2);
    cv::imshow(m_feedName, m_currentFrame);
}

void Camera::mark(const cv::Point& loc, const cv::Scalar& color) {
    cv::line(m_currentFrame, cv::Point(loc.x - 20, loc.y), cv::Point(loc.x + 20, loc.y), color, 2);
    cv::line(m_currentFrame, cv::Point(loc.x, loc.y - 20), cv::Point(loc.x, loc.y + 20), color, 2);
    cv::imshow(m_feedName, m_currentFrame); //show the original image
}

void Camera::circle(const cv::Point& center, int radius, const cv::Scalar& color) {
    cv::circle(m_currentFrame, center, radius, color, 2);
    cv::imshow(m_feedName, m_currentFrame); //show the original image
}

void Camera::markGaps(const Gap& leftGap, const Gap& rightGap) {
	cv::line(m_currentFrame, cv::Point(leftGap.lowerLeft.x, leftGap.lowerLeft.y), cv::Point(leftGap.lowerRight.x, leftGap.lowerRight.y), cv::Scalar(255, 0, 0), 2);
}

void Camera::mouseClick(int x, int y) {
    m_currentClick = (m_currentClick + 1) % 3;
    m_clicks[m_currentClick] = cv::Point(x, y);
    switch (m_currentClick) {
        case 0:
            mark(m_clicks[m_currentClick], cv::Scalar(255, 0, 0)); // first left
            break;
        case 1:
            mark(m_clicks[m_currentClick], cv::Scalar(0, 255, 0)); // then right
            break;
        case 2:
            mark(m_clicks[m_currentClick], cv::Scalar(0, 0, 255)); // then top
            break;
    }

    if (m_currentClick == 2) {
        m_bottomLeftBoundary = m_clicks[0];
        m_bottomRightBoundary = m_clicks[1];
        m_screenHeight = m_bottomLeftBoundary.y - m_clicks[2].y;
        m_screenWidth = m_bottomRightBoundary.x - m_bottomLeftBoundary.x;
        boundariesKnown = true;
        saveBoundaries();
    }
}

void Camera::saveBoundaries() const {
    std::ofstream ofile;
    ofile.open("boundaries.txt", std::ios::out);
    ofile << m_bottomLeftBoundary.x << "\n" << m_bottomLeftBoundary.y << "\n";
    ofile << m_bottomRightBoundary.x << "\n" << m_bottomRightBoundary.y << "\n";
    ofile << m_screenHeight << std::endl; // flush the stream
    ofile.close();
}

void Camera::loadBoundaries() {
    std::ifstream infile{"boundaries.txt", std::ios::in};
    if (infile.is_open()) {
        int x, y;

        infile >> x;
        infile >> y;
        m_bottomLeftBoundary = cv::Point{x, y};

        infile >> x;
        infile >> y;
        m_bottomRightBoundary = cv::Point{x, y};
        m_screenWidth = m_bottomRightBoundary.x - m_bottomLeftBoundary.x;

        infile >> m_screenHeight;

        infile.close();
        boundariesKnown = true;
    } else {
        boundariesKnown = false;
    }
}
