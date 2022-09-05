#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>

#include "physicalArm.hpp"
#include "driver.hpp"
#include "display.hpp"
#include "Recording.hpp"
#include "WebCam.hpp"
#include "ScreenCapture.hpp"
#include "simulatedArm.hpp"
#include "constants.hpp"

void markGap(const Gap& gap, VideoFeed& display) {
    display.mark(display.positionToPixel(gap.lowerLeft), cv::Scalar(255, 0, 0));
    display.mark(display.positionToPixel(gap.lowerRight), cv::Scalar(255, 0, 0));
    display.mark(display.positionToPixel(gap.upperLeft), cv::Scalar(0, 0, 255));
    display.mark(display.positionToPixel(gap.upperRight), cv::Scalar(0, 0, 255));
}

int main(int argc, char** argv) {
    Recording recording;
    Display *X11display = XOpenDisplay(nullptr);

    if (!X11display) {
        std::cerr << "Cannot initialize the X11 display\n";
        exit(EXIT_FAILURE);
    }

    RAIICloser closer([X11display](){ XCloseDisplay(X11display);});
    ScreenCapture screen(X11display);
    VideoFeed display(screen);
    SimulatedArm arm(997, 1545, X11display);

    // WebCam camera;
    // VideoFeed display(camera);
    // VideoFeed display(recording);
    // recording.load(display);
    // PhysicalArm arm(true);
    Driver driver{arm, display};
    bool humanDriving = true;

    try {
        bool recordFeed = false; // TODO use Recording state
        // time at start of recording
        // OR
        // time at start of playback
        TimePoint currentFrameStart{};

        cv::Mat thresholdedBird;
        cv::Mat thresholdedWorld;
//        auto prevT = TimePoint().time_since_epoch().count(); // for calibration
        while (true) {
            if (recordFeed) {
                recording.record(display.getCurrentFrame().clone());
            }

            TimePoint captureStart = toTime(std::chrono::system_clock::now());
            display.captureFrame();
            TimePoint captureEnd = toTime(std::chrono::system_clock::now());

            display.threshold(thresholdedBird, thresholdedWorld);

            if (!display.boundariesKnown()) {
                cv::waitKey(1);
                display.show();
                continue;
            }

            FeatureDetector detector{thresholdedWorld, thresholdedBird, display};

//            for calibrating motion constants
//            std::optional<Position> birdPos = detector.findBird();
//            if (birdPos && recording.getState() == Recording::PLAYBACK) {
//                auto t = TimePoint(recording.m_frames[recording.m_currentPlaybackFrame].first).time_since_epoch().count();
//                if (t != prevT) {
//                    std::cout << "actual y at time " << t << ": " << birdPos->y.val << std::endl;
//                    prevT = t;
//                }
//                if (t == 384) {
//                    driver.predictJump(recording.m_frames, recording.m_currentPlaybackFrame, detector);
//                    break;
//                }
//            }

            std::optional<Position> birdPos = detector.findBird();
            std::pair<std::optional<Gap>, std::optional<Gap>> gaps;
            if (birdPos) {
                // TODO driver will call detector again, so the results might differ or at least delay the whole thing - pass results to driver?
                display.circle(birdPos.value(), BIRD_RADIUS, CV_BLUE);

                gaps = detector.findGapsAheadOf(birdPos.value());
                assert(!gaps.second || gaps.first); // detecting the right but not the left gap would be unexpected

                if (gaps.first) {
                    markGap(gaps.first.value(), display);
                }
                if (gaps.second) {
                    markGap(gaps.second.value(), display);
                }
            }

            if (!humanDriving) {
                driver.drive(birdPos, gaps, captureStart, captureEnd);
            }

            // driver will mark some objects in the frame so only display the frame once it's done
            display.show();

            const char key = cv::waitKey(1);
            if (key == 27) {
                if (recordFeed) {
                    std::cout << "Saving feed" << std::endl;
                    recording.save(display);
                }
                std::cout << "Exiting" << std::endl;
                break;
            } else if (key == 32 && humanDriving) { // space
                arm.tap();
            } else if (key == 'a') {
                if (birdPos) {
                    std::cout << "Switching to automatic" << std::endl;
                    driver.takeOver(birdPos.value());
                    humanDriving = false;
                } else {
                    std::cout << "Switch to automatic FAILED - can't find the bird" << std::endl;
                }
            } else if (key == 'm') {
                std::cout << "Switching to manual" << std::endl;
                humanDriving = true;
            } else if (key == 'r') {
                recordFeed = !recordFeed;
                if (!recordFeed) {
                    std::cout << "Saving feed" << std::endl;
                    recording.save(display);
                } else {
                    std::cout << "Recording feed" << std::endl;
                    recording.startRecording();
                }
            } else if (key == 's') {
                cv::imwrite("screen" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".jpg", display.getCurrentFrame());
                cv::imwrite("screen" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".jpg", thresholdedBird + thresholdedWorld);
            }
        }
    } catch (std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }

    return 0;
}
