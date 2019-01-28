#pragma once

#include <chrono>

struct Speed {
    Speed() = default;
    Speed(float distance, std::chrono::milliseconds time) : distance{distance}, time{time} {}

    /// speed * time = distance
    float operator*(std::chrono::milliseconds duration) const {
        assert(distance != 0 && time != std::chrono::milliseconds{});
        return distance * static_cast<float>(duration.count()) / time.count();
    }

    float distance;
    std::chrono::milliseconds time;
};
