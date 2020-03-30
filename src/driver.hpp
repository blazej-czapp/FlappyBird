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

class Driver {
    // it takes approx 1.632s to cover the unit distance, speed is per millisecond
    static constexpr Speed HORIZONTAL_SPEED{{1.f/1632}};

    // constants calibrated using calibration_recording.xml and calibration_boundaries.txt (the boundaries were from
    // a different recording and the screen is a touch wider than the boundaries, may need to tweak that and recalibrate)
    static constexpr Speed JUMP_SPEED{{-0.00139}}; // tap just sets a new vertical speed, may need some more calibration (increasing abs. value)
    static constexpr Speed TERMINAL_VELOCITY{{0.00198}};
    static constexpr Acceleration GRAVITY{{0.00000451}}; // position grows down, gravity is positive

    static constexpr Distance BIRD_RADIUS{0.05f};

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
    static constexpr const Time::duration TIME_QUANTUM{100};

    // simplifies calculations (canSucceed()) if we can compute events between time quanta independently
    static_assert(TIME_QUANTUM > Arm::TAP_DELAY);

    void markGap(const Gap& gap) const;

    Distance m_groundLevel;

    Arm& m_arm;
    Display& m_disp;
    Time m_lastTapped;
//    Position m_birdPosAtLastTap; // TODO use for debug to compare predicted and actual

    Speed projectVerticalSpeed(Speed startingSpeed, Time::duration deltaT) const;
    Motion predictMotion(Motion motionNow, Time::duration deltaT) const;

    /// lastTap is time of actual physical tap, not just tap intent (i.e. it should take arm delay into account)
    template <typename Predicate>
    bool canSucceed(Motion motion, Time now, Time lastTap, Predicate hasCrashed) const {
        // this could be some score of how good the clearance is rather than just a bool
        if (hasCrashed(motion.position)) {
            return false;
        }

        if (motion.position.x > m_disp.pixelXToPosition(m_disp.getRightBoundary())) {
            return true;
        }

        // depth-first search
        // TODO we try to tap first - maybe we can speed up the search with some heuristic, e.g. tap first if the next gap
        //      is above the bird, otherwise try not tapping first
        // try tapping if we're past cooldown
        if (now - lastTap > Arm::TAP_COOLDOWN) {
            // project to the point of actual tap
            Motion atTap = predictMotion(motion, Arm::TAP_DELAY);
            // compute motion after the tap until the next time quantum (with the new speed after tap)
            Motion atNextQuantum = predictMotion(atTap.with(JUMP_SPEED), TIME_QUANTUM - Arm::TAP_DELAY);
            if (canSucceed(atNextQuantum, now + TIME_QUANTUM, now + Arm::TAP_DELAY, hasCrashed)) {
                return true;
            }
        }

        // if tap doesn't lead to a success or arm is still on cooldown, try not tapping
        return canSucceed(predictMotion(motion, TIME_QUANTUM), now + TIME_QUANTUM, lastTap, hasCrashed);
    }
};
