#include "Recording.hpp"

const std::string Recording::RECORDING_FILE = "recording.xml";
const std::string Recording::FRAMES_KEY = "frames";
const std::string Recording::TIMESTAMPS_KEY = "timestamps";

const TimePoint Recording::NO_FRAME_START = TimePoint::min();