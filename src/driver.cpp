#include "driver.hpp"

#include <iostream>
#include <math.h>
#include <deque>
#include <chrono>

#include "util.hpp"

Driver::Driver(Arm& arm, Display& cam) : m_arm{arm}, m_disp{cam} {}
//        m_horizontalMoveInQuantum(cam.getScreenWidth() * (double{TIME_QUANTUM.count()} / TIME_ACROSS_SCREEN.count())),
//        m_jumpSpeed(cam.getScreenHeight() * RELATIVE_JUMP_HEIGHT, JUMP_DURATION),
//        m_fallSpeed(cam.getScreenHeight(), TIME_TO_FALL_WHOLE_SCREEN),
//        m_groundLevel(cam.getGroundLevel()) {}

void Driver::markGap(const Gap& gap) const {
    m_disp.mark(gap.lowerLeft, cv::Scalar(255, 0, 0));
    m_disp.mark(gap.lowerRight, cv::Scalar(255, 0, 0));
    m_disp.mark(gap.upperLeft, cv::Scalar(0, 0, 255));
    m_disp.mark(gap.upperRight, cv::Scalar(0, 0, 255));
}

void Driver::drive(const FeatureDetector& detector) {
    if (!m_disp.boundariesKnown) {
        return;
    }

    cv::Point birdPos{detector.findBird()};
    if (birdPos.x == -1 || birdPos.y == -1) {
        return; // not initialised yet
    }

    m_disp.circle(birdPos, m_disp.getScreenWidth() * Driver::RADIUS, CV_BLUE);

    //TODO make more robust
    int x = birdPos.x + 20; // so we don't pick up the bird
    int y = birdPos.y;
	
    int gapX, gapYUpper, gapYLower;

    Gap leftGap, rightGap;
    int noOfGaps = detector.findGapsAheadOf(x, leftGap, rightGap);

    if (noOfGaps > 0) {
        markGap(leftGap);
    }
    if (noOfGaps == 2) {
        markGap(rightGap);
    }

    if (noOfGaps == 0) {
        // presumably the very beginning - maybe maintain altitude
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

std::shared_ptr<Driver::State> Driver::tapped(std::shared_ptr<Driver::State> state, std::chrono::system_clock::time_point when) const {
    assert(state->canTap(when));
    cv::Point nextPosition(state->position.x + m_horizontalMoveInQuantum, state->position.y - m_jumpSpeed * TIME_QUANTUM);

    return std::make_shared<State>(state, nextPosition, when);
}

std::shared_ptr<Driver::State> Driver::notTapped(std::shared_ptr<Driver::State> state, std::chrono::system_clock::time_point when) const {
    if (when - state->lastTap < JUMP_DURATION) {
        std::chrono::milliseconds flyUpFor{std::min((JUMP_DURATION - (when - state->lastTap)).count(), TIME_QUANTUM.count())};
        std::chrono::milliseconds flyDownFor = TIME_QUANTUM - flyUpFor;
        int heightDiff = (m_jumpSpeed * flyUpFor) - (m_fallSpeed * flyDownFor);
        // assuming that a tap causes the bird to drive up at a constant speed for JUMP_DURATION,
        // this may be too big an assumption
        cv::Point nextPosition(state->position.x + m_horizontalMoveInQuantum, state->position.y - heightDiff);
        return std::make_shared<State>(state, nextPosition, state->lastTap);
    }
    cv::Point nextPosition(state->position.x + m_horizontalMoveInQuantum, state->position.y + m_fallSpeed * TIME_QUANTUM);
    std::make_shared<State>(state, nextPosition, state->lastTap);
}

bool Driver::State::canTap(std::chrono::system_clock::time_point now) {
    return now - lastTap > Arm::TAP_COOLDOWN;
}

bool Driver::hitGround(const State& state) const {
    return state.position.y > m_groundLevel;
}


//// Bird::State

Driver::State::State(cv::Point position, bool tapped) : position{position}, rootTapped{tapped} {
//TODO last time default is fine, right?
}

// TODO make timeOfLastTap into timeOfLastTap?
Driver::State::State(std::shared_ptr<State>& parent, cv::Point position,
                     std::chrono::system_clock::time_point timeOfLastTap) :
    parent(parent), position(position), lastTap(timeOfLastTap), rootTapped(parent->rootTapped) {}

//bool Driver::State::hasCrashed(int noOfGaps, const Gap& left, const Gap& right) {
//    bool result = noOfGaps > 0 ? collidesWith(left) : false;
//    result |= noOfGaps > 1 ? collidesWith(right) : false;
//    return result || hitGround();
//}
//
//bool Driver::State::collidesWith(const Gap& gap) {
//    return m_position.x >= gap.lowerLeft.x && m_position.x <= gap.lowerRight.x && (m_position.y > gap.lowerLeft.y || m_position.y < gap.upperLeft.y);
//}

