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

// it takes approx 1.632s to cover the unit distance, speed is per millisecond
constexpr Speed HORIZONTAL_SPEED{{1.f/1632}};

// constants calibrated using calibration_recording.xml and calibration_boundaries.txt (the boundaries were from
// a different recording and the screen is a touch wider than the boundaries, may need to tweak that and recalibrate)
constexpr Speed JUMP_SPEED{{-0.00139}}; // tap just sets a new vertical speed, may need some more calibration (increasing abs. value)
constexpr Speed TERMINAL_VELOCITY{{0.00198}};
constexpr Acceleration GRAVITY{{0.00000451}}; // position grows down, gravity is positive

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
    static constexpr const std::chrono::milliseconds TIME_QUANTUM{100};

    void markGap(const Gap& gap) const;

    Distance m_groundLevel;

    Arm& m_arm;
    Display& m_disp;

    struct State {
        State(std::shared_ptr<State>&& parent,
              Motion motion,
              Time time,
              Time lastTap,
              bool rootTapped) : parent(parent), motion(motion), time(time), lastTap(lastTap), rootTapped(rootTapped) {}

        State(std::shared_ptr<State>&& parent,
              Motion motion,
              Time time,
              Time lastTap) : State(std::move(parent), motion, time, lastTap, parent->rootTapped) {}

        Motion calculateMotionAt(Time when) const;

        bool canTap(Time when) const;
        bool hasCrashed(int noOfGaps, const Gap& left, const Gap& right) const;
        bool collidesWith(const Gap& gap) const;

        bool rootTapped; // so we know whether to tap or not after we find a good path

        std::shared_ptr<State> parent;
        Motion motion;
        Time time;
        Time lastTap;
    };

    std::shared_ptr<State> tapped(std::shared_ptr<State> parent, Time when) const;
    std::shared_ptr<State> notTapped(std::shared_ptr<State> parent, Time when) const;
    bool hitGround(const State& state) const;
};
