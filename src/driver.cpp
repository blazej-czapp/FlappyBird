#include "driver.hpp"

#include "util.hpp"
#include "constants.hpp"

#include <iostream>
#include <deque>
#include <chrono>

void markGap(const Gap& gap, VideoFeed& display) {
    display.mark(display.positionToPixel(gap.lowerLeft), cv::Scalar(255, 0, 0));
    display.mark(display.positionToPixel(gap.lowerRight), cv::Scalar(255, 0, 0));
    display.mark(display.positionToPixel(gap.upperLeft), cv::Scalar(0, 0, 255));
    display.mark(display.positionToPixel(gap.upperRight), cv::Scalar(0, 0, 255));
}

Speed Driver::projectVerticalSpeed(Speed startingSpeed, TimePoint::duration deltaT) {
    return startingSpeed + GRAVITY * deltaT;
}

Motion Driver::predictMotion(Motion startingMotion, TimePoint::duration deltaT) {
    assert(startingMotion.verticalSpeed <= TERMINAL_VELOCITY);
    Speed projectedSpeed = projectVerticalSpeed(startingMotion.verticalSpeed, deltaT);

    if (projectedSpeed > TERMINAL_VELOCITY) {
        // If projected speed exceeds TV, we need to find how long it's been the case (tvPeriod), rewind to that moment
        // and apply constant speed from then. We want tvPeriod such that:
        // projectedSpeed - GRAVITY * tvPeriod = TERMINAL_VELOCITY
        // so:
        const TimePoint::duration tvPeriod(static_cast<int>((projectedSpeed - TERMINAL_VELOCITY).val.val / GRAVITY.speed.val.val));

        // if we started at terminal velocity, the period should be equal to deltaT, otherwise it should be less
        assert(TimePoint::duration(0) <= tvPeriod && tvPeriod <= deltaT);

        // overall average speed is the weighted average of avg. speed under acceleration and the terminal velocity
        // period that (maybe) follows (proportional to the time each lasted)
        const Speed averageSpeedUnderAcceleration = (startingMotion.verticalSpeed
                                                     + projectVerticalSpeed(startingMotion.verticalSpeed, deltaT - tvPeriod)
                                                    ) / 2;
        const Speed averageSpeed = averageSpeedUnderAcceleration * (static_cast<float>((deltaT - tvPeriod).count()) / deltaT.count())
                             + TERMINAL_VELOCITY * (static_cast<float>(tvPeriod.count()) / deltaT.count());

        return {{startingMotion.position.x + HORIZONTAL_SPEED * deltaT, startingMotion.position.y + averageSpeed * deltaT},
                TERMINAL_VELOCITY};
    } else {
        const Speed averageSpeed = (startingMotion.verticalSpeed + projectedSpeed) / 2;
        return {{startingMotion.position.x + HORIZONTAL_SPEED * deltaT, startingMotion.position.y + averageSpeed * deltaT},
                projectedSpeed};
    }
}

Driver::Driver(Arm& arm, VideoFeed& cam) : m_arm{arm}, m_disp{cam},
        m_groundLevel(cam.pixelYToPosition(cam.getGroundLevel())), m_lastAction{Action::ANY} {
    // simplifies calculations (bestBestClearance()) if we can compute events between time quanta independently
    //TODO should throw
    assert(SIMULATION_TIME_QUANTUM > m_arm.tapDelay());
    }

void Driver::takeOver(Position birdPos) {
    // tap immediately so we know when the last tap happened
    m_arm.tap();
    m_lastTapped = toTime(std::chrono::system_clock::now()) + m_arm.tapDelay() + BIRD_TAP_DELAY; // + BIRD_TAP_DELAY
}

std::optional<Distance> Driver::pipeClearance(const Gap& gap, const Position& pos) {
    // inside the gap?
    if (pos.x > gap.lowerLeft.x && pos.x < gap.lowerRight.x) {
        const auto gapAbove = (pos.y - BIRD_RADIUS) - gap.upperLeft.y;
        const auto gapBelow = gap.lowerLeft.y - (pos.y + BIRD_RADIUS);

        if (gapAbove < SAFETY_BUFFER || gapBelow < SAFETY_BUFFER) {
            return {};
        } else {
            return std::min(gapAbove, gapBelow);
        }
    } else {
        const Distance smallestClearance = std::min(pos - gap.lowerLeft,
                                                    std::min(pos - gap.lowerRight,
                                                             std::min(pos - gap.upperLeft,
                                                                      pos - gap.upperRight)))
                                            - BIRD_RADIUS;

        if (smallestClearance < SAFETY_BUFFER) {
            return {};
        } else {
            return smallestClearance;
        }
    }
}

std::optional<Distance> Driver::minClearance(Position pos,
                                             const std::pair<std::optional<Gap>,
                                             std::optional<Gap>>& gaps) const {

    if (pos.y + BIRD_RADIUS + GROUND_SAFETY_BUFFER > m_groundLevel) {
        return {};
    }

    assert(!gaps.second || gaps.first);

    if (gaps.first) {
        std::optional<Distance> firstGapClearance = pipeClearance(gaps.first.value(), pos);
        if (firstGapClearance) {

            if (gaps.second) {
                std::optional<Distance> secondGapClearance = pipeClearance(gaps.second.value(), pos);

                if (secondGapClearance) {
                    return std::min(firstGapClearance.value(), secondGapClearance.value());
                } else {
                    return {};
                }
            } else {
                return firstGapClearance;
            }
        } else {
            return {};
        }
    } else {
        return Distance{std::numeric_limits<float>::max()};
    }
}

Driver::Action Driver::bestAction(Motion motion,
                                  TimePoint::duration sinceLastTap,
                                  const std::pair<std::optional<Gap>, std::optional<Gap>>& gaps) const {
    return bestActionR(motion, sinceLastTap, gaps, Distance{std::numeric_limits<float>::max()}).second;
}

std::pair<Distance, Driver::Action>
Driver::bestActionR(Motion motion,
                    TimePoint::duration sinceLastTap,
                    const std::pair<std::optional<Gap>, std::optional<Gap>>& gaps,
                    Distance nearestMissSoFar) const {
    const std::optional<Distance> currentClearance = minClearance(motion.position, gaps);
    if (!currentClearance) {
        // we've crashed into something
        return {Distance{0}, Action::NONE};
    }

    if (motion.position.x > m_disp.pixelXToPosition(m_disp.getRightBoundary())) {
        // we successfully reached the right hand edge of the screen, whatever action brought us here is fine
        return {nearestMissSoFar, Action::ANY};
    }

    std::pair<Distance, Action> bestIfTap{Distance{0}, Action::NONE};
    // depth-first search
    // try tapping if we're past cooldown
    if (sinceLastTap > m_arm.liftDelay()) {
        // project to the point of actual tap
        const Motion atTap = predictMotion(motion, m_arm.tapDelay() + BIRD_TAP_DELAY);
        // then, compute motion from the tap until the next time quantum (with the new speed from tap)
        const Motion atNextQuantum = predictMotion(atTap.with(JUMP_SPEED), SIMULATION_TIME_QUANTUM - m_arm.tapDelay() - BIRD_TAP_DELAY);
        bestIfTap = bestActionR(atNextQuantum, SIMULATION_TIME_QUANTUM - m_arm.tapDelay() - BIRD_TAP_DELAY, gaps,
                                std::min(nearestMissSoFar, currentClearance.value()));
    }

    // now try not tapping
    std::pair<Distance, Action> bestIfNoTap = bestActionR(predictMotion(motion, SIMULATION_TIME_QUANTUM), sinceLastTap + SIMULATION_TIME_QUANTUM,
                                                          gaps, std::min(nearestMissSoFar, currentClearance.value()));

    // Whichever action we choose, the best nearest clearance overall is going to be the smallest of:
    //  - currentClearance
    //  - nearestMissSoFar
    //  - nearest clearance of whichever action we choose
    const Distance smallestIncludingNow = std::min(currentClearance.value(), nearestMissSoFar);
    if (bestIfTap.first > bestIfNoTap.first) {
        assert(bestIfTap.second != Action::NONE); // distance would be 0 otherwise
        return {std::min(smallestIncludingNow, bestIfTap.first), Action::TAP};
    } else if (bestIfTap.first < bestIfNoTap.first) {
        assert(bestIfNoTap.second != Action::NONE); // distance would be 0 otherwise
        return {std::min(smallestIncludingNow, bestIfNoTap.first), Action::NO_TAP};
    } else {
        if (bestIfTap.first == Distance{0}) {
            // if tap and no-tap are equal and both zero, there's no good path
            return {Distance{0}, Action::NONE};
        } else {
            // otherwise just pick arbitrarily
            return {std::min(smallestIncludingNow, bestIfNoTap.first), Action::NO_TAP};
        }
    }
}

void Driver::drive(std::optional<Position> birdPos, std::pair<std::optional<Gap>, std::optional<Gap>> gaps,
                   TimePoint captureStart, TimePoint captureEnd) {

    assert(!gaps.second || gaps.first);
    if (!m_disp.boundariesKnown() || !birdPos || !gaps.first) {
        return;
    }

    // It would be nice to take current time as argument but finding the bird and calculating path takes time (although
    // I haven't measured). For greatest accuracy of the resulting m_lastTapped, let's get our own time from the clock.
    // I guess we could take the clock as argument for testability, but there are no tests anyway :P
    TimePoint now = toTime(std::chrono::system_clock::now());
    // We know where the bird is right now, we're only interested in computing the current speed.
    // We know when we last tapped and what the speed was at that point (JUMP_SPEED) so we can compute the new speed and
    // just overwrite the position with the detected one.

    assert(captureStart < now);
    assert(m_lastTapped <= captureStart); // capture start must be before now

    const TimePoint captureTime = captureStart + m_disp.postCaptureProcessingTime();
    // predict speed at capture time, apply detected position
    const Motion captureStartMotion = predictMotion(Motion{Position{}, JUMP_SPEED}, captureTime - m_lastTapped).with(birdPos.value());
    // correct position by projecting forward by feature detection delay
    const Motion startingMotion = predictMotion(captureStartMotion, now - captureEnd); // captureStart?

    if (m_lastAction == Action::NONE) {
        m_disp.filledCircle(startingMotion.position, BIRD_RADIUS, CV_CYAN);
    } else {
        // this is where we think the bird really is at `now`
        m_disp.filledCircle(startingMotion.position, BIRD_RADIUS, CV_BLUE);
    }

    if (m_lastTapped >= captureStart) {
        std::cout << "TAP PENDING\n";
        return; // tap still pending
    }

    // similarly to the bird, the pipes would have moved (left) on screen during image processing, so simulate
    // that before path finding
    if (gaps.first) {
        gaps.first->shiftHorizontally(-Distance{HORIZONTAL_SPEED * (now - captureStart)});
        markGap(gaps.first.value(), m_disp);
    }
    if (gaps.second) {
        gaps.second->shiftHorizontally(-Distance{HORIZONTAL_SPEED * (now - captureStart)});
        markGap(gaps.second.value(), m_disp);
    }

    const auto best = bestAction(startingMotion, now - m_lastTapped, gaps);

    if (best == Action::TAP) {
        m_arm.tap();
        // arm.tap() starts a new thread which does the tap so let's assume it exits immediately  and so tap delay
        // starts now
        // TODO keep BIRD_TAP_DELAY here?
        m_lastTapped = toTime(std::chrono::system_clock::now()) + m_arm.tapDelay() + BIRD_TAP_DELAY;
    }

    static int c = 1;
    if (best == Action::NONE) {
        std::cout << "COULDN'T FIND PATH! " << ++c << std::endl;
        m_disp.filledCircle(startingMotion.position, BIRD_RADIUS, CV_CYAN);
    }

    m_lastAction = best;
}

// predicting from recording to calibrate motion constants
void Driver::predictPosition(const std::vector<std::pair<TimePoint::duration, cv::Mat>>& recording,
                             size_t startFrame,
                             const FeatureDetector& detector,
                             Speed initialSpeed) const {
    std::optional<Position> birdPos = detector.findBird();
    if (!birdPos) {
        return;
    }

    TimePoint::duration startTime(recording[startFrame].first);

    for (size_t i = startFrame + 1; i < recording.size() - 1; ++i) {
        const auto timeDelta = recording[i].first - recording[startFrame].first;
        const Motion projected = predictMotion(Motion{birdPos.value(), initialSpeed}, recording[i].first - startTime);

        std::cout << "predicted y at time " << TimePoint(recording[i].first).time_since_epoch().count() << " " << projected.position.y.val << std::endl;
    }
}

// used for calibrating the JUMP_SPEED, GRAVITY and TERMINAL_VELOCITY constants, presumably from a moment of jump
// (in practice, the true jump start can happen between frames so worth trying this a few times)
void Driver::predictJump(const std::vector<std::pair<TimePoint::duration, cv::Mat>>& recording,
                         size_t startFrame,
                         const FeatureDetector& detector) {
    predictPosition(recording, startFrame, detector, JUMP_SPEED);
}


// used for calibrating the GRAVITY and TERMINAL_VELOCITY constants, presumably from a standstill point just before freefall
// (in practice, the true peak can happen between frames so using an estimate for the true speed at the actual frame)
void Driver::predictFreefall(const std::vector<std::pair<TimePoint::duration, cv::Mat>>& recording,
                             size_t startFrame,
                             const FeatureDetector& detector) {
    predictPosition(recording, startFrame, detector, Speed{0.00011});
}
