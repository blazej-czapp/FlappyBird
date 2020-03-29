#ifndef FLAPPYBIRD_RECORDING_HPP
#define FLAPPYBIRD_RECORDING_HPP

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>

#include "display.hpp"

class Recording : public VideoSource {
    const static std::string RECORDING_FILE;
    const static std::string FRAMES_KEY;
    const static std::string TIMESTAMPS_KEY;
    const static Time NO_FRAME_START;

    /// the only integral type OpenCV's serialization supports is int
    using TimestampSerializationT = int;
public:
    enum State {
        IDLE,
        PLAYBACK,
        RECORDING
    };

    State getState() const {
        return m_state;
    }

    bool load(Display& display) {
        assert(m_state == IDLE);

        std::cout << "Loading recording from " << RECORDING_FILE << std::endl;

        reset();

        cv::FileStorage fs(RECORDING_FILE, cv::FileStorage::READ);

        if (!fs.isOpened()) {
            std::cerr << "Couldn't open file: " << RECORDING_FILE << std::endl;
            return false;
        }

        cv::FileNode frames = fs[FRAMES_KEY];
        if (frames.type() != cv::FileNode::SEQ)
        {
            std::cerr << "Frames not a cv::FileNode::SEQ" << std::endl;
            return false;
        }

        if (frames.size() < 2) {
            std::cerr << "Recording must have at least two frames (found " << frames.size() << ")" << std::endl;
            return false;
        }

        const std::chrono::milliseconds placeholder{};
        cv::FileNodeIterator framesEnd = frames.end();
        for (cv::FileNodeIterator it = frames.begin(); it != framesEnd; ++it)
        {
            cv::Mat temp;
            *it >> temp;
            m_frames.push_back(std::make_pair(placeholder, std::move(temp)));
        }

        cv::FileNode timestamps = fs[TIMESTAMPS_KEY];
        if (timestamps.type() != cv::FileNode::SEQ)
        {
            std::cerr << "Timestamps not a cv::FileNode::SEQ" << std::endl;
            return false;
        }

        if (timestamps.size() < 2) {
            std::cerr << "Recording must have at least two timestamps (found " << timestamps.size() << ")" << std::endl;
            return false;
        }

        if (timestamps.size() != frames.size()) {
            std::cerr << "Recording must have equal number of frames and timestamps. Found " << frames.size()
                      << " frames and " << timestamps.size() << " timestamps." << std::endl;
            return false;
        }

        cv::FileNodeIterator timestampsEnd = timestamps.end();
        cv::FileNodeIterator timestampsBegin = timestamps.begin();
        for (cv::FileNodeIterator it = timestampsBegin; it != timestampsEnd; ++it)
        {
            TimestampSerializationT timestamp;
            *it >> timestamp;
            m_frames[it - timestampsBegin].first = std::chrono::milliseconds{timestamp};
        }

        // scene boundaries are loaded as they were at the time of recording
        display.deserialise(fs);

        cv::namedWindow("Playback speed");
        cv::createTrackbar("Speed", "Playback speed", &m_playbackSpeed, 100);

        m_state = PLAYBACK;

        m_loaded = true;
        return true;
    }

    void save(const Display& display) {
        assert(m_state == RECORDING);
        std::cout << "Saving feed to: " << RECORDING_FILE << std::endl;

        if (m_frames.size() < 2) {
            std::cerr << "Cannot save recording, it must have at least two frames (this one has " << m_frames.size() << ")" << std::endl;
            return;
        }
        cv::FileStorage fs(RECORDING_FILE, cv::FileStorage::WRITE);

        // scene boundaries are also saved so we don't have to manually select them at load time
        display.serialise(fs);

        fs << "frames" << "[";
        for (const auto& frame : m_frames) {
            fs << frame.second;
        }
        fs << "]";

        fs << "timestamps" << "[";
        for (const auto& frame : m_frames) {
            assert(frame.first.count() < std::numeric_limits<TimestampSerializationT>::max());
            fs << static_cast<TimestampSerializationT>(frame.first.count());
        }
        fs << "]";

        reset();
    }

    void startRecording() {
        // TODO this should be a separate member (separate for recording, separate for playback). Do we need a separate
        // type for each? Note this isn't updated when recording frames, so it's really "start of recording", not of
        // current frame.
        m_currentFrameStart = std::chrono::system_clock::now();
        m_state = RECORDING;
    }

    void record(const cv::Mat& frame) {
        assert(m_state == RECORDING);
        m_frames.push_back(std::make_pair(
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - m_currentFrameStart),
                frame));
    }

    // Recording captures a frame by advancing the tape if next frame is due
    const cv::Mat& captureFrame() override {
        assert(m_loaded);

        // if this is the first frame we're capturing, start counting time from here
        if (m_currentFrameStart == NO_FRAME_START) {
            m_currentFrameStart = std::chrono::system_clock::now();
            m_currentPlaybackFrame = 0;
        }

        Time now = std::chrono::system_clock::now();
        std::chrono::milliseconds currentFrameElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_currentFrameStart);

        // This is interesting - by multiplying a std::chrono::milliseconds (which is std::chrono::duration<int64_t>)
        // by a float, we obtain a std::chrono::duration<float>. If m_playbackSpeed is 0, betweenFrameDelta.count() is
        // inf and everything behaves as expected (we get a pause).
        std::chrono::milliseconds betweenFrameDelta =
                std::chrono::duration_cast<std::chrono::milliseconds>((m_frames[m_currentPlaybackFrame + 1].first
                                                                       - m_frames[m_currentPlaybackFrame].first)
                                                                      * (100.f / m_playbackSpeed));

        if (currentFrameElapsed > betweenFrameDelta) {
            // last frame isn't displayed (we don't know its duration) - it's only used to determine
            // the duration of the penultimate frame
            m_currentPlaybackFrame = (m_currentPlaybackFrame + 1) % (m_frames.size() - 1);
            m_currentFrameStart = now - (currentFrameElapsed - betweenFrameDelta);
        }

        return m_frames[m_currentPlaybackFrame].second;
    }

    void reset() {
        m_state = IDLE;
        m_frames.clear();
        m_currentFrameStart = NO_FRAME_START;
        m_currentPlaybackFrame = 0;
    }

//private: commented out for calibration
    // time is from the start of the recording
    std::vector<std::pair<std::chrono::milliseconds, cv::Mat>> m_frames;
    Time m_currentFrameStart = NO_FRAME_START;
    size_t m_currentPlaybackFrame = 0;
    State m_state = State::IDLE;
    int m_playbackSpeed = 100; // percent
    bool m_loaded = false;
};

#endif //FLAPPYBIRD_RECORDING_HPP
