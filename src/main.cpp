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

int main(int argc, char** argv) {
    Recording recording;
    Display *X11display = XOpenDisplay(nullptr);

    if (!X11display) {
        std::cerr << "Cannot initialize the X11 display\n";
        exit(EXIT_FAILURE);
    }

    RAIICloser closer([X11display](){ XCloseDisplay(X11display);});
    ScreenCapture screen(X11display);
    SimulatedArm arm(997, 1545, X11display);

    VideoFeed display(screen);

    // VideoFeed display(recording);
    // recording.load(display);

    // WebCam camera;
    // VideoFeed display(camera);

    // PhysicalArm arm(true);
    Driver driver{arm, display};
    bool humanDriving = false;

    cv::Mat thresholdedBird;
    cv::Mat thresholdedWorld;
    FeatureDetector detector{display};

    try {
        bool recordFeed = false; // TODO use Recording state
        // time at start of recording
        // OR
        // time at start of playback
        TimePoint currentFrameStart{};

        auto prevT = TimePoint().time_since_epoch().count(); // for calibration
        while (true) {
            if (!display.boundariesKnown()) {
                cv::waitKey(1);
                display.show();
                continue;
            }

            TimePoint frameStart = toTime(std::chrono::system_clock::now());

            TimePoint captureStart = toTime(std::chrono::system_clock::now());
            display.captureFrame(); // 2-6ms on X11 (emulator)
            TimePoint captureEnd = toTime(std::chrono::system_clock::now());

            std::optional<Position> birdPos;
            if (recordFeed) {
                recording.record(display.getCurrentFrame());
            } else {
                detector.process(display.getCurrentFrame());

                // for calibrating motion constants
                if (recording.getState() == Recording::PLAYBACK) {
                    std::optional<Position> birdPosCal = detector.findBird();
                    if (birdPosCal) {
                        std::pair<std::optional<Gap>, std::optional<Gap>> gaps = detector.findGapsAheadOf(
                            birdPosCal.value());
                        display.circle(birdPosCal.value(), BIRD_RADIUS, CV_CYAN);
                        if (gaps.first) {
                            std::cout << "gap x: " << gaps.first->lowerLeft.x.val << std::endl;
                        }
                        auto t = TimePoint(recording.m_frames[recording.m_currentPlaybackFrame].first)
                                     .time_since_epoch()
                                     .count();
                        if (t < prevT) {
                        // if (t == 1226) {
                            break;
                        }
                        if (t != prevT) {
                            std::cout << "actual y at time " << t << " " << birdPosCal->y.val << std::endl;
                            prevT = t;
                        }
                        if (t == 49) {
                            driver.predictJump(recording.m_frames, recording.m_currentPlaybackFrame,
                                               detector);
                            // break;
                        }

                    }
                }

                if (!recordFeed) {
                    birdPos = detector.findBird();
                    std::pair<std::optional<Gap>, std::optional<Gap>> gaps;
                    if (birdPos) {
                        if (!recordFeed) {
                            // don't mark anything during recording, it will confuse the detector working off the recording
                            display.circle(Position{birdPos.value().x, Coordinate{birdPos.value().y.val}},
                                           BIRD_RADIUS, CV_BLUE);
                        }

                        gaps = detector.findGapsAheadOf(birdPos.value());
                        assert(!gaps.second || gaps.first); // detecting the right but not the left gap would be unexpected
                    }

                    if (!humanDriving) {
                        driver.drive(birdPos, gaps, captureStart, captureEnd);
                    }
                }
            }

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
                // if (recording.getState() != Recording::PLAYBACK) {
                //     recording.startRecording();
                //     recordFeed = true;
                // }
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
            } else { // any other key or nothing, useful when we set the wait to 0 (wait forever), so we can advance
                     // frame manually (when playing a recording)
                TimePoint frameEnd = toTime(std::chrono::system_clock::now());
                // std::cout << frameEnd.time_since_epoch().count() - frameStart.time_since_epoch().count() << std::endl;

                continue;
            }
        }
    } catch (std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }

    return 0;
}
