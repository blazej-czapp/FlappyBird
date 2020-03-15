#pragma once

#include <deque>
#include <memory>
#include <chrono>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "arm.hpp"
#include "display.hpp"
#include "featureDetector.hpp"
#include "units.hpp"

// all times in seconds
constexpr std::chrono::milliseconds TIME_ACROSS_SCREEN{1800};
constexpr std::chrono::milliseconds TIME_QUANTUM{100};
constexpr Speed HORIZONTAL_SPEED{{1.f/TIME_ACROSS_SCREEN.count()}};

// tap just sets a new vertical speed, it's not an acceleration
constexpr Speed JUMP_SPEED{{-0.01}}; // TODO random value, measure it

constexpr Acceleration GRAVITY{{10}}; // position grows down, gravity is positive

class Driver {
public:
    Driver(Arm& arm, Display& cam);
    void drive(const FeatureDetector&);

    static constexpr Distance BIRD_RADIUS{0.05f};

//    std::vector<std::pair<size_t, Position>> predictFreefall(std::vector<std::pair<std::chrono::milliseconds, cv::Mat>> recording,
//                                                             size_t currentFrame,
//                                                             const FeatureDetector& detector);

private:
    void markGap(const Gap& gap) const;

    Distance m_groundLevel;

    Arm& m_arm;
    Display& m_disp;

    struct State {
        /// used to initialize root
        State(Position position, bool tapped);
        /// used to initialize children
        State(std::shared_ptr<State>& parent,
              Position position,
              std::chrono::system_clock::time_point time,
              Speed verticalSpeed,
              std::chrono::system_clock::time_point timeOfLastTap);

        Position calculatePositionAt(std::chrono::system_clock::time_point when) const;
        Speed calculateVerticalSpeedAt(std::chrono::system_clock::time_point when) const;

        bool canTap(std::chrono::system_clock::time_point when) const;
        bool hasCrashed(int noOfGaps, const Gap& left, const Gap& right) const;
        bool collidesWith(const Gap& gap) const;

        bool rootTapped; // so we know whether to tap or not after we find a good path

        std::shared_ptr<State> parent;
        Position position;
        std::chrono::system_clock::time_point time;
        Speed verticalSpeed; // negative when falling
        std::chrono::system_clock::time_point lastTap;
    };

    std::shared_ptr<State> tapped(std::shared_ptr<State> parent, std::chrono::system_clock::time_point when) const;
    std::shared_ptr<State> notTapped(std::shared_ptr<State> parent, std::chrono::system_clock::time_point when) const;
    bool hitGround(const State& state) const;
};
