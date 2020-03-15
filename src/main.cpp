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
    std::reference_wrapper<VideoSource> source = camera;
    Display display(source);
//    recording.load(display);
    Arm arm;
    Driver driver{arm, display};
    bool humanDriving = true;

    try {
        bool recordFeed = false;
        // time at start of recording
        // OR
        // time at start of playback
        std::chrono::system_clock::time_point currentFrameStart{};
        size_t currentPlaybackFrame = 0;

        bool playback = false;

        cv::Mat thresholdedBird;
        cv::Mat thresholdedWorld;
        while (true) {
            if (recordFeed) {
                recording.record(display.getCurrentFrame().clone());
            }
            display.captureFrame();

            display.threshold(thresholdedBird, thresholdedWorld);

            //TODO pulling it out of scope below for calibration, put it back when done
            FeatureDetector detector{thresholdedWorld, thresholdedBird, display};
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

            } else if (key == 'p' && playback) {
                //driver.predictFreefall(recording, currentPlaybackFrame, detector);
            }
        }
    } catch (std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }

    arm.deactivate();
    return 0;
}
