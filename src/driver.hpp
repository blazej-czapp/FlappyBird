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

public:
    Driver(Arm& arm, VideoFeed& cam);
    void drive(std::optional<Position> birdPos, std::pair<std::optional<Gap>, std::optional<Gap>> gaps,
               TimePoint captureStart, TimePoint captureEnd);
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
    enum class Action {
        TAP,
        NO_TAP,
        ANY,
        NONE
    };

    Coordinate m_groundLevel;

    Arm& m_arm;
    VideoFeed& m_disp;
    TimePoint m_lastTapped;
    Action m_lastAction;

    std::optional<Distance> minClearance(Position pos, const std::pair<std::optional<Gap>,
                                         std::optional<Gap>>& gaps) const;

    // should these be free functions? we'd need to make the constants public or pass them directly
    static Speed projectVerticalSpeed(Speed startingSpeed, TimePoint::duration deltaT);
    static Motion predictMotion(Motion motionNow, TimePoint::duration deltaT);
    // no value if crashed
    static std::optional<Distance> pipeClearance(const Gap& gap, const Position& pos);

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
