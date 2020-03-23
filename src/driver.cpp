#include "driver.hpp"

#include <iostream>
#include <math.h>
#include <deque>
#include <chrono>

#include "util.hpp"

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

    // just set the new upward speed, no acceleration involved
    return std::make_shared<State>(parent, parent->calculatePositionAt(when), when, JUMP_SPEED, when);
}

std::shared_ptr<Driver::State> Driver::notTapped(std::shared_ptr<Driver::State> parent, Time when) const {
    return std::make_shared<State>(parent, parent->calculatePositionAt(when), when,
                                   parent->calculateVerticalSpeedAt(when), parent->lastTap);
}

bool Driver::State::canTap(Time when) const {
    return when - lastTap > Arm::TAP_COOLDOWN;
}

bool Driver::hitGround(const State& state) const {
    return m_groundLevel < state.position.y;
}

Speed projectVerticalSpeed(Speed startingSpeed, Time now, Time when) {
    return startingSpeed + GRAVITY * std::chrono::duration_cast<std::chrono::milliseconds>(when - now);
}

// predicting from recording to calibrate motion constants
void predictMotion(const std::vector<std::pair<std::chrono::milliseconds, cv::Mat>>& recording,
                   size_t startFrame,
                   const FeatureDetector& detector,
                   Speed initialSpeed) {
    std::optional<Position> birdPos = detector.findBird();
    if (!birdPos) {
        return;
    }

    Time startTime(recording[startFrame].first);
    std::optional<size_t> terminalVelocityFrame;
    std::optional<Distance> terminalVelocityPosition;

    for (size_t i = startFrame + 1; i < recording.size() - 1; ++i) {
        Distance projectedY;
        if (terminalVelocityFrame) {
            // constant speed at terminal velocity
            assert(terminalVelocityPosition);
            const auto timeDelta = recording[i].first- recording[terminalVelocityFrame.value()].first;

            projectedY = terminalVelocityPosition.value() + TERMINAL_VELOCITY * timeDelta;
        } else {
            // accelerating under gravity
            const auto timeDelta = recording[i].first - recording[startFrame].first;
            const Speed finalSpeed = projectVerticalSpeed(initialSpeed, startTime, Time(recording[i].first));

            Speed averageSpeed = (initialSpeed + finalSpeed) / 2;
            projectedY = birdPos.value().y + averageSpeed * timeDelta;

            if (finalSpeed.distance.val > TERMINAL_VELOCITY.distance.val) {
                terminalVelocityFrame = i;
                terminalVelocityPosition = projectedY;
            }
        }

        std::cout << "" << Time(recording[i].first).time_since_epoch().count() << " " << projectedY.val << std::endl;
    }
}

// used for calibrating the JUMP_SPEED, GRAVITY and TERMINAL_VELOCITY constants, presumably from a moment of jump
// (in practice, the true jump start can happen between frames so worth trying this a few times)
void Driver::predictJump(const std::vector<std::pair<std::chrono::milliseconds, cv::Mat>>& recording,
                         size_t startFrame,
                         const FeatureDetector& detector) {
    predictMotion(recording, startFrame, detector, JUMP_SPEED);
}


// used for calibrating the GRAVITY and TERMINAL_VELOCITY constants, presumably from a standstill point just before freefall
// (in practice, the true peak can happen between frames so using an estimate for the true speed at the actual frame)
void Driver::predictFreefall(const std::vector<std::pair<std::chrono::milliseconds, cv::Mat>>& recording,
                             size_t startFrame,
                             const FeatureDetector& detector) {
    predictMotion(recording, startFrame, detector, Speed{0.00011});
}

// TODO initial vertical speed can be computed based on time of last tap
// TODO have some debug check that current (detected) bird pos aligns with the computed one (at root)
Driver::State::State(Position position, bool tapped) : position{position}, rootTapped{tapped} {}

Driver::State::State(std::shared_ptr<State>& parent, Position position, Time time,
                     Speed verticalSpeed, Time timeOfLastTap) :
    rootTapped(parent->rootTapped), parent(parent), position(position), time(time), verticalSpeed(verticalSpeed),
    lastTap(timeOfLastTap) {}

Speed Driver::State::calculateVerticalSpeedAt(Time when) const {
    return verticalSpeed + GRAVITY * std::chrono::duration_cast<std::chrono::milliseconds>(when - time);
}

Position Driver::State::calculatePositionAt(Time when) const {
    const Speed finalSpeed = projectVerticalSpeed(verticalSpeed, time, when);
    const Speed averageSpeed = (verticalSpeed + finalSpeed) / 2;
    return {position.x + HORIZONTAL_SPEED * std::chrono::duration_cast<std::chrono::milliseconds>(when - time),
            position.y + averageSpeed * TIME_QUANTUM};
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

