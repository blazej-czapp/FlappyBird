#include "Recording.hpp"

const std::string Recording::RECORDING_FILE = "recording.xml";
const std::string Recording::FRAMES_KEY = "frames";
const std::string Recording::TIMESTAMPS_KEY = "timestamps";

const std::chrono::system_clock::time_point Recording::NO_FRAME_START = std::chrono::system_clock::time_point::min();