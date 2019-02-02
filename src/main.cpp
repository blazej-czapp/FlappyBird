#include <string>
#include <iostream>
#include <vector>
#include <chrono>
#include <memory>

#include "opencv2/core/core.hpp"

#include "arm.hpp"
#include "driver.hpp"
#include "display.hpp"
#include "util.hpp"

const static std::string RECORDING_FILE = "recording.xml";
const static std::string FRAMES_KEY = "frames";
const static std::string TIMESTAMPS_KEY = "timestamps";

using Recording = std::vector<std::pair<std::chrono::milliseconds, cv::Mat>>;

/// the only integral type OpenCV's serialization supports is int
using TimestampSerializationT = int;

int playbackSpeed = 100; // percent

void saveRecording(const Recording& recording, const Display& display) {
    if (recording.size() < 2) {
        std::cerr << "Cannot save recording, it must have at least two frames (this one has " << recording.size() << ")" << std::endl;
        return;
    }
    cv::FileStorage fs(RECORDING_FILE, cv::FileStorage::WRITE);

    display.serialise(fs);

    fs << "frames" << "[";
    for (const auto& frame : recording) {
        fs << frame.second;
    }
    fs << "]";

    fs << "timestamps" << "[";
    for (const auto& frame : recording) {
        assert(frame.first.count() < std::numeric_limits<TimestampSerializationT>::max());
        fs << static_cast<TimestampSerializationT>(frame.first.count());
    }
    fs << "]";
}

bool loadRecording(Recording& out, Display& display) {
    cv::FileStorage fs(RECORDING_FILE, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        std::cerr << "Couldn't open file: " << RECORDING_FILE << std::endl;
        return false;
    }

    cv::FileNode frames = fs[FRAMES_KEY];
    if (frames.type() != cv::FileNode::SEQ)
    {
        std::cerr << "Frames not a sequence" << std::endl;
        return false;
    }

    if (frames.size() < 2) {
        std::cerr << "Recording must have at least two frames (this has " << frames.size() << " frames)" << std::endl;
        return false;
    }

    const std::chrono::milliseconds placeholder{};
    cv::FileNodeIterator framesEnd = frames.end();
    for (cv::FileNodeIterator it = frames.begin(); it != framesEnd; ++it)
    {
        cv::Mat temp;
        *it >> temp;
        out.push_back(std::make_pair(placeholder, std::move(temp)));
    }

    cv::FileNode timestamps = fs[TIMESTAMPS_KEY];
    if (timestamps.type() != cv::FileNode::SEQ)
    {
        std::cerr << "Frames not a sequence" << std::endl;
        return false;
    }

    if (timestamps.size() < 2) {
        std::cerr << "Recording must have at least two frames (this has " << timestamps.size() << " timestamps)" << std::endl;
        return false;
    }

    cv::FileNodeIterator timestampsEnd = timestamps.end();
    cv::FileNodeIterator timestampsBegin = timestamps.begin();
    for (cv::FileNodeIterator it = timestampsBegin; it != timestampsEnd; ++it)
    {
        TimestampSerializationT timestamp;
        *it >> timestamp;
        out[it - timestampsBegin].first = std::chrono::milliseconds{timestamp};
    }

    display.deserialise(fs);

    return true;
}

int main(int argc, char** argv) {
    Display display;
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
        Recording recording;

        bool playback = false;

        cv::Mat thresholdedBird;
        cv::Mat thresholdedWorld;
        while (true) {
            if (playback) {
                auto now = std::chrono::system_clock::now();
                auto frameElapsed = now - currentFrameStart;
                auto frameDelta = (recording[currentPlaybackFrame + 1].first - recording[currentPlaybackFrame].first) *
                                  (100.f / playbackSpeed);

                if (frameElapsed > frameDelta) {
                    // last frame isn't displayed (we don't know its duration) - it's only used to determine
                    // the duration of the penultimate frame
                    currentPlaybackFrame = (currentPlaybackFrame + 1) % (recording.size() - 1);
                    currentFrameStart = now - std::chrono::duration_cast<std::chrono::milliseconds>(frameElapsed -
                                                                                                    frameDelta);
                }

                display.playback(recording[currentPlaybackFrame].second);
            } else {
                display.capture();
                if (recordFeed) {
                    recording.push_back(std::make_pair(
                            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() -
                                                                                  currentFrameStart),
                            display.getCurrentFrame().clone()));
                }
            }

            display.threshold(thresholdedBird, thresholdedWorld);

            if (!humanDriving) {
                FeatureDetector detector{thresholdedWorld, thresholdedBird, display};
                driver.drive(detector);
            }

            // driver will mark some objects in the frame so only display the frame once it's done
            display.show();

            const char key = cv::waitKey(15);
            if (key == 27) {
                std::cout << "Exiting" << std::endl;
                if (recordFeed) {

                }
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
                std::cout << "Recording feed" << std::endl;
                currentFrameStart = std::chrono::system_clock::now();

                if (!recordFeed) {
                    std::cout << "Saving feed to: " << RECORDING_FILE << std::endl;
                    saveRecording(recording, display);
                    recording.clear();
                }
            } else if (key == 'l') {
                if (!playback) {
                    std::cout << "Loading recording from " << RECORDING_FILE << std::endl;
                    cv::namedWindow("Playback speed");
                    cv::createTrackbar("Speed", "Playback speed", &playbackSpeed, 100);
                    if (loadRecording(recording, display)) {
                        playback = true;
                        currentFrameStart = std::chrono::system_clock::now();
                        currentPlaybackFrame = 0;
                    }
                }
            }
        }
    } catch (std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }

    arm.deactivate();
    return 0;
}
