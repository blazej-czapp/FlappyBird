#pragma once

#include <chrono>

using Time = std::chrono::system_clock::time_point;

// these are not really units, couldn't think of a better name

/// Unit is is the width of the rating/score box under the GAME OVER sign (inclusive of the outermost pixels, measured
/// towards the bottom when it's more straight with this camera). This should be the most robust with respect to
/// changing camera position and resolution than screen width.
struct Distance {
    Distance operator*(float m) const {
        return Distance{val * m};
    }

    Distance operator+(const Distance &other) const {
        return Distance{val + other.val};
    }

    Distance operator-(const Distance &other) const {
        return Distance{val - other.val};
    }

    Distance operator+=(const Distance &other) {
        val += other.val;
    }

    bool operator<(const Distance& other) const {
        return val < other.val;
    }

    float val;
};

struct Position {
    Distance x; // proportion of screen width from left of active area
    Distance y; // proportion of screen WIDTH (see comment for Distance) from top of active area
};

/// per millisecond, positive is down
struct Speed {
    /// speed * time = distance
    Distance operator*(std::chrono::milliseconds duration) const {
        return distance * duration.count();
    }

    Speed operator+(const Speed &other) const {
        return Speed{distance + other.distance};
    }

    Speed operator/(float denominator) const {
        return Speed{distance.val / denominator};
    }

    Distance distance;
};

/// per millisecond
struct Acceleration {
    constexpr Acceleration(Speed speed) : speed{speed} {}

    Speed operator*(std::chrono::milliseconds delta) const {
        return Speed{speed.distance * delta.count()};
    }

    Speed speed;
};

