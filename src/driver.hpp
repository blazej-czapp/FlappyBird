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
#include "util.hpp"

class Driver {
    // it takes approx 1.632s to cover the unit distance, speed is per millisecond
    static constexpr Speed HORIZONTAL_SPEED{{1.f/1632}};

    // constants calibrated using calibration_recording.xml and calibration_boundaries.txt (the boundaries were from
    // a different recording and the screen is a touch wider than the boundaries, may need to tweak that and recalibrate)

    // tap just sets a new vertical speed, this may need some more calibration (together with gravity) - because tap
    // happens between frames, I had to estimate the speed at tap by fitting speed and gravity at frame
    static constexpr Speed JUMP_SPEED{{-0.00139}};
    static constexpr Speed TERMINAL_VELOCITY{{0.00198}}; // this should be quite accurate
    static constexpr Acceleration GRAVITY{{0.00000451}}; // position grows down, gravity is positive

    static constexpr Distance BIRD_RADIUS{0.06f};

    // "grow" pipes by this much in all directions for collision detection (should yield safer paths but may cause no
    // path to be found if too large)
    static constexpr Distance SAFETY_BUFFER{0.02};

public:
    Driver(Arm& arm, Display& cam);
    void drive(const FeatureDetector&);
    void takeOver(/*Position birdPos*/);

    void predictFreefall(const std::vector<std::pair<Time::duration, cv::Mat>>& recording,
                         size_t startFrame,
                         const FeatureDetector& detector);

    void predictJump(const std::vector<std::pair<Time::duration, cv::Mat>>& recording,
                     size_t startFrame,
                     const FeatureDetector& detector);

    void predictPosition(const std::vector<std::pair<Time::duration, cv::Mat>>& recording,
                         size_t startFrame,
                         const FeatureDetector& detector,
                         Speed initialSpeed) const;

private:
    static constexpr const Time::duration TIME_QUANTUM{150};

    // simplifies calculations (canSucceed()) if we can compute events between time quanta independently
    static_assert(TIME_QUANTUM > Arm::TAP_DELAY);

    void markGap(const Gap& gap) const;

    Coordinate m_groundLevel;

    Arm& m_arm;
    Display& m_disp;
    Time m_lastTapped;
//    Position m_birdPosAtLastTap; // TODO use for debug to compare predicted and actual

    bool hasCrashed(Position pos, const std::pair<std::optional<Gap>, std::optional<Gap>>& gaps) const;

    // should these be free functions? we'd need to make the constants public or pass them directly
    static Speed projectVerticalSpeed(Speed startingSpeed, Time::duration deltaT);
    static Motion predictMotion(Motion motionNow, Time::duration deltaT);
    static bool hitsPipe(const Gap& gap, const Position& pos, const Distance& radius);

    /// Given current motion, can we steer the bird through all visible pipes?
    /// @param sinceLastTap time from the actual physical tap, not since we last issued a tap request (i.e. takes
    ///                     arm delay into account)
    bool canSucceed(Motion motion,
                    Time::duration sinceLastTap,
                    const std::pair<std::optional<Gap>, std::optional<Gap>>& gaps) const;
};
