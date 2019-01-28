#pragma once

#include <deque>
#include <memory>
#include <chrono>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "arm.hpp"
#include "display.hpp"
#include "featureDetector.hpp"
#include "speed.hpp"

// all times in seconds
constexpr std::chrono::milliseconds TIME_ACROSS_SCREEN{1800};
constexpr std::chrono::milliseconds TIME_TO_FALL_WHOLE_SCREEN{1000};
constexpr std::chrono::milliseconds TIME_QUANTUM{100};
constexpr float RELATIVE_JUMP_HEIGHT = 240.0f / 2000;
constexpr std::chrono::milliseconds JUMP_DURATION{300};

class Driver {
public:
    Driver(Arm& arm, Display& cam);
    void drive(const FeatureDetector&);

    static constexpr float RADIUS{0.05f}; // as proportion of screen width

private:
    void markGap(const Gap& gap) const;

    unsigned m_horizontalMoveInQuantum;
    Speed m_jumpSpeed;
    Speed m_fallSpeed;
    int m_groundLevel;

    Arm& m_arm;
    Display& m_disp;

    struct State {
        /// used to initialize root
        State(cv::Point position, bool tapped);
        /// used to initialize children
        State(std::shared_ptr<State>& parent, cv::Point position, std::chrono::system_clock::time_point timeOfLastTap);

        bool canTap(std::chrono::system_clock::time_point now);
        bool hasCrashed(int noOfGaps, const Gap& left, const Gap& right);
        bool collidesWith(const Gap& gap);

        bool rootTapped;

        std::shared_ptr<State> parent;
        cv::Point position;
        std::chrono::system_clock::time_point lastTap;
    };

    std::shared_ptr<State> tapped(std::shared_ptr<State> state, std::chrono::system_clock::time_point when) const;
    std::shared_ptr<State> notTapped(std::shared_ptr<State> state, std::chrono::system_clock::time_point when) const;
    bool hitGround(const State& state) const;
};
