#pragma once

#include <chrono>
#include <cmath>

// Doing this as `struct TimePoint : public std::chrono::time_point...` would be nice because we could create some convenient
// constructors but has the disadvantage that e.g. operator=() (and potentially others) has to be implemented explicitly
// for time_point argument - otherwise the default one takes TimePoint& of which time_point is the base class and so cannot
// be downcast.
using TimePoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>;

// these are not really units, couldn't think of a better name

/// Unit is is the width of the rating/score box under the GAME OVER sign (inclusive of the outermost pixels, measured
/// towards the bottom when it's more straight with this camera). This should be the most robust with respect to
/// changing camera position and resolution than screen width.
struct Distance {
    Distance operator*(float m) const {
        return Distance{val * m};
    }

    Distance operator/(float m) const {
        return Distance{val / m};
    }

    Distance operator+(const Distance &other) const {
        return Distance{val + other.val};
    }

    Distance operator-(const Distance &other) const {
        return Distance{val - other.val};
    }

    Distance operator+=(const Distance &other) {
        val += other.val;
        return *this;
    }

    Distance operator-=(const Distance &other) {
        val -= other.val;
        return *this;
    }

    Distance operator-() const {
        return Distance{-val};
    }

    bool operator<(const Distance& other) const {
        return val < other.val;
    }

    bool operator<=(const Distance& other) const {
        return !(val > other.val);
    }

    bool operator>(const Distance& other) const {
        return *this != other && !(*this < other);
    }

    bool operator>=(const Distance& other) const {
        return !(*this < other);
    }

    bool operator==(const Distance& other) const {
        return val == other.val;
    }

    bool operator!=(const Distance& other) const {
        return !(*this == other);
    }

    float val;
};

struct Coordinate {
    float val;

    bool operator==(const Coordinate& other) const {
        return val == other.val;
    }

    bool operator!=(const Coordinate& other) const {
        return !(*this == other);
    }

    // it's not meaningful, mathematically, to add two points
    Distance operator+(const Coordinate& other) = delete;

    Distance operator-(const Coordinate& other) const {
        return Distance{val - other.val};
    }

    Coordinate operator+(const Distance& dist) const {
        return Coordinate{val + dist.val};
    }

    Coordinate operator-(const Distance& dist) const {
        return Coordinate{val - dist.val};
    }

    void operator-=(const Distance& dist) {
        val -= dist.val;
    }

    void operator+=(const Distance& dist) {
        val += dist.val;
    }

    bool operator<(const Coordinate& other) const {
        return val < other.val;
    }

    bool operator<=(const Coordinate& other) const {
        return !(val > other.val);
    }

    bool operator>(const Coordinate& other) const {
        return *this != other && !(*this < other);
    }

    bool operator>=(const Coordinate& other) const {
        return !(*this < other);
    }
};

struct Position {
    Coordinate x; // distance from the left edge of the viewport
    Coordinate y; // distance from the top edge of the viewport

    Distance operator-(const Position& other) const {
        return { static_cast<float>(std::sqrt(std::pow((other.x - x).val, 2) + std::pow((other.y - y).val, 2))) };
    }
};

/// per millisecond, positive is down
struct Speed {
    /// speed * time = distance
    Distance operator*(TimePoint::duration duration) const {
        return val * duration.count();
    }

    Speed operator*(float factor) const {
        return {val * factor};
    }

    Speed operator+(const Speed &other) const {
        return Speed{val + other.val};
    }

    Speed operator-(const Speed &other) const {
        return Speed{val - other.val};
    }

    Speed operator/(float denominator) const {
        return Speed{val.val / denominator};
    }

    bool operator<(Speed other) const {
        return val < other.val;
    }

    bool operator<=(Speed other) const {
        return val <= other.val;
    }

    bool operator>(Speed other) const {
        return val > other.val;
    }

    bool operator>=(Speed other) const {
        return val >= other.val;
    }

    bool operator==(Speed other) const {
        return val == other.val;
    }

    bool operator!=(Speed other) const {
        return !(*this == other);
    }

    Distance val;
};

/// per millisecond
struct Acceleration {
    constexpr Acceleration(Speed speed) : speed{speed} {}

    Speed operator*(TimePoint::duration delta) const {
        return Speed{speed.val * delta.count()};
    }

    Speed speed;
};

struct Motion {
    Position position;
    Speed verticalSpeed;

    Motion with(Speed newSpeed) const {
        return Motion{position, newSpeed};
    }

    Motion with(Position newPosition) const {
        return Motion{newPosition, verticalSpeed};
    }
};
