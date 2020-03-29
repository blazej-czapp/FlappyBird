#include "driver.hpp"

#include <iostream>
#include <math.h>
#include <deque>
#include <chrono>

#include "util.hpp"

namespace {

Speed projectVerticalSpeed(Speed startingSpeed, Time::duration deltaT) {
    return startingSpeed + GRAVITY * deltaT;
}

Motion predictMotion(Position posNow, Speed speedNow, Time::duration deltaT) {
    assert(speedNow <= TERMINAL_VELOCITY);
    Speed projectedSpeed = projectVerticalSpeed(speedNow, deltaT);

    if (projectedSpeed > TERMINAL_VELOCITY) {
        // If projected speed exceeds TV, we need to find how long it's been the case (t), rewind to that moment
        // and apply constant speed from then. We want t such that:
        // projectedSpeed - GRAVITY * t = TERMINAL_VELOCITY
        // so:
        std::chrono::milliseconds t(static_cast<int>((projectedSpeed - TERMINAL_VELOCITY).val.val / GRAVITY.speed.val.val));

        // if we started at terminal velocity, the period should be equal to deltaT, otherwise it should be less
        assert(std::chrono::milliseconds{0} <= t && t <= deltaT);

        // overall average speed is the weighted average of avg. speed under acceleration and the terminal velocity
        // period that (maybe) follows (proportional to the time each lasted)
        Speed averageSpeedUnderAcceleration = (speedNow + projectVerticalSpeed(speedNow, deltaT - t)) / 2;
        Speed averageSpeed = averageSpeedUnderAcceleration * (static_cast<float>((deltaT - t).count()) / deltaT.count())
                             + TERMINAL_VELOCITY * (static_cast<float>(t.count()) / deltaT.count());

        return {{posNow.x + HORIZONTAL_SPEED * deltaT, posNow.y + averageSpeed * deltaT},
                TERMINAL_VELOCITY};
    } else {
        const Speed averageSpeed = (speedNow + projectedSpeed) / 2;
        return {{posNow.x + HORIZONTAL_SPEED * deltaT, posNow.y + averageSpeed * deltaT},
                projectedSpeed};
    }
}

}

Driver::Driver(Arm& arm, Display& cam) : m_arm{arm}, m_disp{cam},
        m_groundLevel(cam.pixelYToPosition(cam.getGroundLevel())) {}

void Driver::markGap(const Gap& gap) const {
    m_disp.mark(gap.lowerLeft, cv::Scalar(255, 0, 0));
    m_disp.mark(gap.lowerRight, cv::Scalar(255, 0, 0));
    m_disp.mark(gap.upperLeft, cv::Scalar(0, 0, 255));
    m_disp.mark(gap.upperRight, cv::Scalar(0, 0, 255));
}

void Driver::drive(const FeatureDetector& detector) {
    if (!m_disp.boundariesKnown()) {
        return;
    }

    std::optional<Position> birdPos = detector.findBird();
    if (!birdPos) {
        return; // not initialised yet
    }

    m_disp.circle(birdPos.value(), Driver::BIRD_RADIUS, CV_BLUE);

    //TODO make more robust
    // scaling x so we don't pick up the bird, the bird is always at the same position horizontally so we don't need
    // to worry about the distance growing
    Position pos{birdPos->x * 1.1, birdPos->y};

    int gapX, gapYUpper, gapYLower;

    std::pair<std::optional<Gap>, std::optional<Gap>> gaps = detector.findGapsAheadOf(birdPos.value());
    assert(!gaps.second || gaps.first); // detecting the right but not the left gap would be unexpected

    if (gaps.first) {
        markGap(gaps.first.value());
    }
    if (gaps.second) {
        markGap(gaps.second.value());
    }

    if (!gaps.first && !gaps.second) {
        // presumably at the beginning - maintain altitude?
        return;
    }

//---------------------------------
    return;
//---------------------------------
//
//    std::deque<State*> pendingStates;
//    // exploring two subtrees options - one with the initial tap and one without
//    pendingStates.push_back(new State(NULL, m_disp, birdPos, m_lastTapped, true));
//    pendingStates.push_back(new State(NULL, m_disp, birdPos, m_lastTapped, false));
//    for (int i = 0; i < TIME_ACROSS_SCREEN / TIME_QUANTUM; i++) {
//        State* s = pendingStates[0];
//        pendingStates.pop_front();
//        std::vector<State *> candidates; // TODO can this slow anything down?
//        if (s->canTap(now)) {
//            candidates.push_back(s->tapped(now));
//        }
//        candidates.push_back(s->notTapped(now));
//
//        for (uint i = 0; i < candidates.size(); i++) {
//            if (candidates[i]->hasCrashed(noOfGaps, leftGap, rightGap)) {
//                candidates[i]->destroy();
//            } else {
//                pendingStates.push_back(candidates[i]);
//            }
//        }
//    }
//
//    // at this point, pendingStates contains tips of successful paths
//    // for now, just pick the first one, later - choose the one that misses all obstacles
//    // by the largest margin
//    if (pendingStates.size() > 0 && pendingStates[0]->rootTapped) {
//        std::cout << "TAPPING - PLANNED" << std::endl;
//        m_arm.tap();
//        m_lastTapped = now;
//    } else {
//        std::cout << "NOT TAPPING - PLANNED" << std::endl;
//    }
}

std::shared_ptr<Driver::State> Driver::tapped(std::shared_ptr<Driver::State> parent, Time when) const {
    assert(parent->canTap(when));

    // TODO this is wrong, motion calculation doesn't take the tap into account
    return std::make_shared<State>(std::move(parent), parent->calculateMotionAt(when), when, when);
}

std::shared_ptr<Driver::State> Driver::notTapped(std::shared_ptr<Driver::State> parent, Time when) const {
    return std::make_shared<State>(std::move(parent), parent->calculateMotionAt(when), when, parent->lastTap);
}

bool Driver::State::canTap(Time when) const {
    return when - lastTap > Arm::TAP_COOLDOWN;
}

bool Driver::hitGround(const State& state) const {
    return m_groundLevel < state.motion.position.y;
}

// predicting from recording to calibrate motion constants
void predictPosition(const std::vector<std::pair<std::chrono::milliseconds, cv::Mat>>& recording,
                   size_t startFrame,
                   const FeatureDetector& detector,
                   Speed initialSpeed) {
    std::optional<Position> birdPos = detector.findBird();
    if (!birdPos) {
        return;
    }

    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> startTime(recording[startFrame].first);

    for (size_t i = startFrame + 1; i < recording.size() - 1; ++i) {
        const auto timeDelta = recording[i].first - recording[startFrame].first;
        const Motion projected = predictMotion(birdPos.value(), initialSpeed, recording[i].first - std::chrono::milliseconds(startTime.time_since_epoch()));

        std::cout << "" << Time(recording[i].first).time_since_epoch().count() << " " << projected.position.y.val << std::endl;
    }
}

// used for calibrating the JUMP_SPEED, GRAVITY and TERMINAL_VELOCITY constants, presumably from a moment of jump
// (in practice, the true jump start can happen between frames so worth trying this a few times)
void Driver::predictJump(const std::vector<std::pair<std::chrono::milliseconds, cv::Mat>>& recording,
                         size_t startFrame,
                         const FeatureDetector& detector) {
    predictPosition(recording, startFrame, detector, JUMP_SPEED);
}


// used for calibrating the GRAVITY and TERMINAL_VELOCITY constants, presumably from a standstill point just before freefall
// (in practice, the true peak can happen between frames so using an estimate for the true speed at the actual frame)
void Driver::predictFreefall(const std::vector<std::pair<std::chrono::milliseconds, cv::Mat>>& recording,
                             size_t startFrame,
                             const FeatureDetector& detector) {
    predictPosition(recording, startFrame, detector, Speed{0.00011});
}

Motion Driver::State::calculateMotionAt(Time when) const {
    return predictMotion(motion.position, motion.verticalSpeed, when - time);
}

//bool Driver::State::hasCrashed(int noOfGaps, const Gap& left, const Gap& right) const {
//    bool result = noOfGaps > 0 ? collidesWith(left) : false;
//    result |= noOfGaps > 1 ? collidesWith(right) : false;
//    return result || hitGround();
//}
//
//bool Driver::State::collidesWith(const Gap& gap) const {
//    return m_position.x >= gap.lowerLeft.x && m_position.x <= gap.lowerRight.x && (m_position.y > gap.lowerLeft.y || m_position.y < gap.upperLeft.y);
//}

