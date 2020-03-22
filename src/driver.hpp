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

// constants calibrated using calibration_recording.xml and calibration_boundaries.txt (the boundaries were from
// a different recording and the screen is a touch wider than the boundaries, may need to tweak that and recalibrate)
// TODO calibrate again to minimise error from "tap before frame capture"
constexpr Speed JUMP_SPEED{{-0.00103}}; // tap just sets a new vertical speed
constexpr Speed TERMINAL_VELOCITY{{0.00151}};
constexpr Acceleration GRAVITY{{0.00000343}}; // position grows down, gravity is positive

class Driver {
public:
    Driver(Arm& arm, Display& cam);
    void drive(const FeatureDetector&);

    static constexpr Distance BIRD_RADIUS{0.05f};

    void predictFreefall(const std::vector<std::pair<std::chrono::milliseconds, cv::Mat>>& recording,
                         size_t startFrame,
                         const FeatureDetector& detector);

    void predictJump(const std::vector<std::pair<std::chrono::milliseconds, cv::Mat>>& recording,
                     size_t startFrame,
                     const FeatureDetector& detector);

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
              Time time,
              Speed verticalSpeed,
              Time timeOfLastTap);

        Position calculatePositionAt(Time when) const;
        Speed calculateVerticalSpeedAt(Time when) const;

        bool canTap(Time when) const;
        bool hasCrashed(int noOfGaps, const Gap& left, const Gap& right) const;
        bool collidesWith(const Gap& gap) const;

        bool rootTapped; // so we know whether to tap or not after we find a good path

        std::shared_ptr<State> parent;
        Position position;
        Time time;
        Speed verticalSpeed; // negative when falling
        Time lastTap;
    };

    std::shared_ptr<State> tapped(std::shared_ptr<State> parent, Time when) const;
    std::shared_ptr<State> notTapped(std::shared_ptr<State> parent, Time when) const;
    bool hitGround(const State& state) const;
};
