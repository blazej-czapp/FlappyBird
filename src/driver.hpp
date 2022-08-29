#pragma once

#include <deque>
#include <memory>
#include <chrono>
#include <variant>
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
    static constexpr Speed JUMP_SPEED{{-0.00130}}; // -0.00126 from experiments
    static constexpr Speed TERMINAL_VELOCITY{{0.00198}}; // this should be quite accurate
    static constexpr Acceleration GRAVITY{{0.00000451}}; // position grows down, gravity is positive

    static constexpr Distance BIRD_RADIUS{0.06f};

    // "grow" pipes by this much in all directions for collision detection (should yield safer paths but may cause no
    // path to be found if too large)
    static constexpr Distance SAFETY_BUFFER{0.00};

public:
    Driver(Arm& arm, VideoFeed& cam);
    void drive(const FeatureDetector&, TimePoint captureStart, TimePoint captureEnd);
    void takeOver(Position birdPos);

    void predictFreefall(const std::vector<std::pair<TimePoint::duration, cv::Mat>>& recording,
                         size_t startFrame,
                         const FeatureDetector& detector);

    void predictJump(const std::vector<std::pair<TimePoint::duration, cv::Mat>>& recording,
                     size_t startFrame,
                     const FeatureDetector& detector);

    void predictPosition(const std::vector<std::pair<TimePoint::duration, cv::Mat>>& recording,
                         size_t startFrame,
                         const FeatureDetector& detector,
                         Speed initialSpeed) const;

private:
    static constexpr const TimePoint::duration TIME_QUANTUM{100};

    void markGap(const Gap& gap) const;

    Coordinate m_groundLevel;

    Arm& m_arm;
    VideoFeed& m_disp;
    TimePoint m_lastTapped;
    Position m_birdPosAtLastTap; // TODO use for debug to compare predicted and actual

    Distance minClearance(Position pos, const std::pair<std::optional<Gap>, std::optional<Gap>>& gaps) const;

    // should these be free functions? we'd need to make the constants public or pass them directly
    static Speed projectVerticalSpeed(Speed startingSpeed, TimePoint::duration deltaT);
    static Motion predictMotion(Motion motionNow, TimePoint::duration deltaT);
    static Distance pipeClearance(const Gap& gap, const Position& pos);

    enum class Action {
        TAP,
        NO_TAP,
        ANY,
        NONE
    };

    Action bestAction(Motion motion,
                      TimePoint::duration sinceLastTap,
                      const std::pair<std::optional<Gap>, std::optional<Gap>>& gaps) const;

    /// Given current motion, how can we steer the bird through all visible pipes? Right now 'best' means 'first one
    /// we can find with depth-first-search'.
    /// @param sinceLastTap time from the actual physical tap, not since we last issued a tap request (i.e. takes
    ///                     arm delay into account)
    /// @returns the action that achieves the greatest nearest-approach to any obstacle and the distance of that
    ///          approach (can be {Distance{0}, NONE} if no path can be found from `motion`)
    std::pair<Distance, Action> bestActionR(Motion motion,
                                            TimePoint::duration sinceLastTap,
                                            const std::pair<std::optional<Gap>, std::optional<Gap>>& gaps,
                                            Distance nearestMissSoFar) const;
};
