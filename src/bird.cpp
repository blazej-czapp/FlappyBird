#include <iostream>
#include <math.h>
#include <deque>
#include "bird.hpp"

Bird::Bird(Arm& arm, Camera& cam) : m_arm(arm), m_cam(cam), m_lastTapped(0) {}

void Bird::fly(const World& world, const cv::Point& pos) {
    if (!m_cam.boundariesKnown) {
        return;
    }

    if (pos.x == -1 || pos.y == -1) {
        return; // not initialised yet
    }

    //TODO make more robust
    int x = pos.x + 20; // so we don't pick up the bird 
    int y = pos.y;
	
    int gapX, gapYUpper, gapYLower;

    Gap leftGap, rightGap;
    int noOfGaps = world.findGapsAheadOf(x, leftGap, rightGap);

    m_cam.markGaps(leftGap, rightGap);
    
    clock_t now = std::clock();
    if (noOfGaps == 0) {
        if (float(now - m_lastTapped) / CLOCKS_PER_SEC >= MAINTAIN_ALTITUDE_TAP_INTERVAL) {
            std::cout << "TAPPING - NO OBSTACLES" << std::endl;
            m_arm.tap();
            m_lastTapped = now;
        }
        return;
    }

//---------------------------------
    return;
//---------------------------------

    std::deque<State*> pendingStates;
    // exploring two subtrees options - one with the initial tap and one without
    pendingStates.push_back(new State(NULL, m_cam, pos, m_lastTapped, true));
    pendingStates.push_back(new State(NULL, m_cam, pos, m_lastTapped, false));
    for (int i = 0; i < TIME_ACROSS_SCREEN / TIME_QUANTUM; i++) {
        State* s = pendingStates[0];
        pendingStates.pop_front();
        std::vector<State *> candidates; // TODO can this slow anything down?
        if (s->canTap(now)) {
            candidates.push_back(s->tapped(now));
        }
        candidates.push_back(s->notTapped(now));

        for (uint i = 0; i < candidates.size(); i++) {
            if (candidates[i]->hasCrashed(noOfGaps, leftGap, rightGap)) {
                candidates[i]->destroy();
            } else {
                pendingStates.push_back(candidates[i]);
            }
        }
    }

    // at this point, pendingStates contains tips of successful paths
    // for now, just pick the first one, later - choose the one that misses all obstacles
    // by the largest margin
    if (pendingStates.size() > 0 && pendingStates[0]->rootTapped) {
        std::cout << "TAPPING - PLANNED" << std::endl;
        m_arm.tap();
        m_lastTapped = now;
    } else {
        std::cout << "NOT TAPPING - PLANNED" << std::endl;
    }

    for (uint i = 0; i < pendingStates.size(); i++) {
        pendingStates[i]->destroy();
    }
}

//// Bird::State

Bird::State::State(State* parent, Camera& cam, cv::Point position, clock_t timeOfLastTap, bool rootTapped) : 
    m_parent(parent), m_cam(cam), m_position(position), m_lastTap(timeOfLastTap), m_childrenCount(0), rootTapped(rootTapped),
    m_horizontalMoveInQuantum(cam.getScreenWidth() * TIME_QUANTUM / TIME_ACROSS_SCREEN),
    m_jumpSpeed(m_cam.getAbsoluteHeight(RELATIVE_JUMP_HEIGHT / JUMP_DURATION)),
    m_fallSpeed(m_cam.getAbsoluteHeight(1.0f) / TIME_TO_FALL_WHOLE_SCREEN) {} // TODO don't recalculate every time?

bool Bird::State::canTap(clock_t now) {
    return now - m_lastTap >= Arm::TAP_COOLDOWN;
}

Bird::State* Bird::State::tapped(clock_t now) {
    assert(canTap(now));
    cv::Point nextPosition(m_position.x + m_horizontalMoveInQuantum, m_position.y - TIME_QUANTUM * m_jumpSpeed);

    m_childrenCount++;
    return new State(this, m_cam, nextPosition, now, rootTapped);
}

Bird::State* Bird::State::notTapped(clock_t now) {
    if (now - m_lastTap <= JUMP_DURATION) {
        unsigned flyUpFor = std::min(JUMP_DURATION - float(now - m_lastTap) / CLOCKS_PER_SEC, TIME_QUANTUM);
        unsigned flyDownFor = TIME_QUANTUM - flyUpFor;
        int heightDiff = (m_jumpSpeed * flyUpFor) - (m_fallSpeed * flyDownFor);
        // assuming that a tap causes the bird to fly up at a constant speed for JUMP_DURATION,
        // this may be too big an assumption
        cv::Point nextPosition(m_position.x + m_horizontalMoveInQuantum, m_position.y - heightDiff);
        m_childrenCount++;
        return new State(this, m_cam, nextPosition, m_lastTap, rootTapped);
    }
    cv::Point nextPosition(m_position.x + m_horizontalMoveInQuantum, m_position.y + TIME_QUANTUM * m_fallSpeed);
    m_childrenCount++;
    return new State(this, m_cam, nextPosition, m_lastTap, rootTapped);
}

void Bird::State::childDied() {
    if (--m_childrenCount == 0) {
        m_parent->childDied();
        delete this;
    }
}

bool Bird::State::hasCrashed(int noOfGaps, const Gap& left, const Gap& right) {
    bool result = noOfGaps > 0 ? collidesWith(left) : false;
    result |= noOfGaps > 1 ? collidesWith(right) : false;
    return result || hitGround();
}

bool Bird::State::collidesWith(const Gap& gap) {
    return m_position.x >= gap.lowerLeft.x && m_position.x <= gap.lowerRight.x && (m_position.y > gap.lowerLeft.y || m_position.y < gap.upperLeft.y);
}

bool Bird::State::hitGround() {
    return m_position.y > m_cam.getGroundLevel();
}

void Bird::State::destroy() {
    m_parent->childDied();
    delete this;
}
