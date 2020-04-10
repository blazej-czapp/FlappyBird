#include "driver.hpp"

#include <iostream>
#include <deque>
#include <chrono>

#include "util.hpp"

Speed Driver::projectVerticalSpeed(Speed startingSpeed, Time::duration deltaT) {
    return startingSpeed + GRAVITY * deltaT;
}

Motion Driver::predictMotion(Motion motionNow, Time::duration deltaT) {
    assert(motionNow.verticalSpeed <= TERMINAL_VELOCITY);
    Speed projectedSpeed = projectVerticalSpeed(motionNow.verticalSpeed, deltaT);

    if (projectedSpeed > TERMINAL_VELOCITY) {
        // If projected speed exceeds TV, we need to find how long it's been the case (tvPeriod), rewind to that moment
        // and apply constant speed from then. We want tvPeriod such that:
        // projectedSpeed - GRAVITY * tvPeriod = TERMINAL_VELOCITY
        // so:
        Time::duration tvPeriod(static_cast<int>((projectedSpeed - TERMINAL_VELOCITY).val.val / GRAVITY.speed.val.val));

        // if we started at terminal velocity, the period should be equal to deltaT, otherwise it should be less
        assert(Time::duration(0) <= tvPeriod && tvPeriod <= deltaT);

        // overall average speed is the weighted average of avg. speed under acceleration and the terminal velocity
        // period that (maybe) follows (proportional to the time each lasted)
        Speed averageSpeedUnderAcceleration = (motionNow.verticalSpeed
                                               + projectVerticalSpeed(motionNow.verticalSpeed, deltaT - tvPeriod)
                                              ) / 2;
        Speed averageSpeed = averageSpeedUnderAcceleration * (static_cast<float>((deltaT - tvPeriod).count()) / deltaT.count())
                             + TERMINAL_VELOCITY * (static_cast<float>(tvPeriod.count()) / deltaT.count());

        return {{motionNow.position.x + HORIZONTAL_SPEED * deltaT, motionNow.position.y + averageSpeed * deltaT},
                TERMINAL_VELOCITY};
    } else {
        const Speed averageSpeed = (motionNow.verticalSpeed + projectedSpeed) / 2;
        return {{motionNow.position.x + HORIZONTAL_SPEED * deltaT, motionNow.position.y + averageSpeed * deltaT},
                projectedSpeed};
    }
}

Driver::Driver(Arm& arm, Display& cam) : m_arm{arm}, m_disp{cam},
        m_groundLevel(cam.pixelYToPosition(cam.getGroundLevel())) {}

void Driver::markGap(const Gap& gap) const {
    m_disp.mark(m_disp.positionToPixel(gap.lowerLeft), cv::Scalar(255, 0, 0));
    m_disp.mark(m_disp.positionToPixel(gap.lowerRight), cv::Scalar(255, 0, 0));
    m_disp.mark(m_disp.positionToPixel(gap.upperLeft), cv::Scalar(0, 0, 255));
    m_disp.mark(m_disp.positionToPixel(gap.upperRight), cv::Scalar(0, 0, 255));
}

void Driver::takeOver(/*Position birdPos*/) {
    // tap immediately so we know when the last tap happened
    m_arm.tap();
    m_lastTapped = toTime(std::chrono::system_clock::now()) + Arm::TAP_DELAY;
//    m_birdPosAtLastTap = birdPos;
}

bool Driver::hitsPipe(const Gap& gap, const Position& pos, const Distance& radius) {
    return pos.x + radius >= (gap.lowerLeft.x - SAFETY_BUFFER)
           && pos.x - radius <= (gap.lowerRight.x + SAFETY_BUFFER)
           && (pos.y + radius >= (gap.lowerLeft.y - SAFETY_BUFFER)
               || pos.y - radius <= (gap.upperLeft.y - SAFETY_BUFFER));
}

bool Driver::hasCrashed(Position pos, const std::pair<std::optional<Gap>, std::optional<Gap>>& gaps) const {
    if (m_groundLevel < pos.y) {
        return true;
    }

    assert(!gaps.second || gaps.first);
    if (gaps.second) {
        return hitsPipe(gaps.first.value(), pos, BIRD_RADIUS) || hitsPipe(gaps.second.value(), pos, BIRD_RADIUS);
    } else if (gaps.first) {
        return hitsPipe(gaps.first.value(), pos, BIRD_RADIUS);
    } else {
        return false;
    }
}

bool Driver::canSucceed(Motion motion,
                        Time::duration sinceLastTap,
                        const std::pair<std::optional<Gap>, std::optional<Gap>>& gaps) const {
    // this could be some score of how good the clearance is rather than just a bool
    if (hasCrashed(motion.position, gaps)) {
        return false;
    }

    if (motion.position.x > m_disp.pixelXToPosition(m_disp.getRightBoundary())) {
        return true;
    }

    // depth-first search
    // TODO we try to tap first - maybe we can speed up the search with some heuristic, e.g. tap first if the next gap
    //      is above the bird, otherwise try not tapping first
    // try tapping if we're past cooldown
    if (sinceLastTap > Arm::TAP_COOLDOWN) {
        // project to the point of actual tap
        Motion atTap = predictMotion(motion, Arm::TAP_DELAY);
        // then, compute motion from the tap until the next time quantum (with the new speed from tap)
        Motion atNextQuantum = predictMotion(atTap.with(JUMP_SPEED), TIME_QUANTUM - Arm::TAP_DELAY);
        if (canSucceed(atNextQuantum, TIME_QUANTUM - Arm::TAP_DELAY, gaps)) {
            return true;
        }
    }

    // if tap doesn't lead to a success or arm is still on cooldown, try not tapping
    return canSucceed(predictMotion(motion, TIME_QUANTUM), sinceLastTap + TIME_QUANTUM, gaps);
}

void Driver::drive(const FeatureDetector& detector) {
    if (!m_disp.boundariesKnown()) {
        return;
    }

    std::optional<Position> birdPos = detector.findBird();
    if (!birdPos) {
        return; // not initialised yet
    }

    m_disp.circle(birdPos.value(), BIRD_RADIUS, CV_BLUE);

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

    // It would be nice to take current time as argument but finding the bird and calculating path takes time (although
    // I haven't measured). For greatest accuracy of the resulting m_lastTapped, let's get our own time from the clock.
    // I guess we could take the clock as argument for testability, but there are no tests anyway :P
    Time now = toTime(std::chrono::system_clock::now());
    // We know where the bird is right now, we're only interested in computing the current speed.
    // We know when we last tapped and what the speed was at that point (JUMP_SPEED) so we can compute the new speed and
    // just overwrite the position with the detected one.
    // TODO store the position at last tap and verify the projected position is close to the detected one
    Position dummyPos;
    Motion startingMotion = predictMotion(Motion{dummyPos, JUMP_SPEED}, now - m_lastTapped).with(birdPos.value());

    if (canSucceed(Motion{birdPos.value(), startingMotion.verticalSpeed}, now - m_lastTapped, gaps)) {
        m_arm.tap();
        // arm.tap() starts a new thread which does the tap so let's assume it exits immediately  and so tap delay
        // starts now
        m_lastTapped = toTime(std::chrono::system_clock::now()) + Arm::TAP_DELAY;
    }
}

// predicting from recording to calibrate motion constants
void Driver::predictPosition(const std::vector<std::pair<Time::duration, cv::Mat>>& recording,
                             size_t startFrame,
                             const FeatureDetector& detector,
                             Speed initialSpeed) const {
    std::optional<Position> birdPos = detector.findBird();
    if (!birdPos) {
        return;
    }

    Time::duration startTime(recording[startFrame].first);

    for (size_t i = startFrame + 1; i < recording.size() - 1; ++i) {
        const auto timeDelta = recording[i].first - recording[startFrame].first;
        const Motion projected = predictMotion(Motion{birdPos.value(), initialSpeed}, recording[i].first - startTime);

        std::cout << "" << Time(recording[i].first).time_since_epoch().count() << " " << projected.position.y.val << std::endl;
    }
}

// used for calibrating the JUMP_SPEED, GRAVITY and TERMINAL_VELOCITY constants, presumably from a moment of jump
// (in practice, the true jump start can happen between frames so worth trying this a few times)
void Driver::predictJump(const std::vector<std::pair<Time::duration, cv::Mat>>& recording,
                         size_t startFrame,
                         const FeatureDetector& detector) {
    predictPosition(recording, startFrame, detector, JUMP_SPEED + Speed{0.00005});
}


// used for calibrating the GRAVITY and TERMINAL_VELOCITY constants, presumably from a standstill point just before freefall
// (in practice, the true peak can happen between frames so using an estimate for the true speed at the actual frame)
void Driver::predictFreefall(const std::vector<std::pair<Time::duration, cv::Mat>>& recording,
                             size_t startFrame,
                             const FeatureDetector& detector) {
    predictPosition(recording, startFrame, detector, Speed{0.00011});
}
