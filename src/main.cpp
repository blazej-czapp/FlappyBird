#include <string>
#include <iostream>
#include <vector>
#include <chrono>
#include <memory>

#include "opencv2/core/core.hpp"

#include "arm.hpp"
#include "driver.hpp"
#include "display.hpp"
#include "Recording.hpp"
#include "WebCam.hpp"

int main(int argc, char** argv) {
    Recording recording;
    WebCam camera;
    Display display(camera);
//    Display display(recording);
//    recording.load(display);
    Arm arm;
    Driver driver{arm, display};
    bool humanDriving = true;

    try {
        bool recordFeed = false; // TODO use Recording state
        // time at start of recording
        // OR
        // time at start of playback
        Time currentFrameStart{};

        cv::Mat thresholdedBird;
        cv::Mat thresholdedWorld;
//        auto prevT = Time().time_since_epoch().count(); // for calibration
        while (true) {
            if (recordFeed) {
                recording.record(display.getCurrentFrame().clone());
            }
            display.captureFrame();

            display.threshold(thresholdedBird, thresholdedWorld);

            //TODO pulling it out of scope below for calibration, put it back when done
            FeatureDetector detector{thresholdedWorld, thresholdedBird, display};

//            for calibrating motion constants
//            std::optional<Position> birdPos = detector.findBird();
//            if (birdPos && recording.getState() == Recording::PLAYBACK) {
//                auto t = Time(recording.m_frames[recording.m_currentPlaybackFrame].first).time_since_epoch().count();
//                if (t != prevT) {
//                    std::cout << "actual y at time " << t << ": " << birdPos->y.val << std::endl;
//                    prevT = t;
//                }
//                if (t == 446000000) {
//                    driver.predictJump(recording.m_frames, recording.m_currentPlaybackFrame, detector);
//                    break;
//                }
//            }

            if (!humanDriving) {
                driver.drive(detector);
            }

            // driver will mark some objects in the frame so only display the frame once it's done
            display.show();

            const char key = cv::waitKey(15);
            if (key == 27) {
                std::cout << "Exiting" << std::endl;
                break;
            } else if (key == 32 && humanDriving) { // space
                std::cout << "space" << std::endl;
                arm.tap();
            } else if (key == 'a') {
                std::cout << "Switching to automatic" << std::endl;
                humanDriving = false;
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
            }
        }
    } catch (std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }

    arm.deactivate();
    return 0;
}
