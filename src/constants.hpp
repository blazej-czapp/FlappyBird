#pragma once

#include "units.hpp"

using namespace std::literals::chrono_literals; // propagate when including this header, not a big deal

/*
 * These should be all constants that control the simulation (e.g. physical constants) plus flight control
 * (e.g. safety margins). There are a few more in featureDetector.cpp - those are fairly stable and shouldn't
 * need too much tinkering.
 * Unit of length is the width of the score box (the one that shows when you crash), including the border
 */

// motion constants obtained from disassembling the game apk
static constexpr Speed HORIZONTAL_SPEED{{0.0006082}};

// tap just sets a new vertical speed
static constexpr Speed JUMP_SPEED{{-0.00144329}};
static constexpr Speed TERMINAL_VELOCITY{{0.002061855}}; // this should be quite accurate
static constexpr Acceleration GRAVITY{{0.00000474226}}; // position grows down, gravity is positive

// "grow" pipes by this much in all directions for collision detection; the idea is to discard candidate
// paths during search that pass too close to the obstacles, saving time evaluating them further (the
// expectation being that we'll find a path that doesn't need to cut it this fine)
// it's a remnant of early experiments when we accepted the first path that didn't crash, not the largest
// clearance one,  I've got a feeling this makes no real difference to performance
static constexpr Distance SAFETY_BUFFER{0.000};

static constexpr Distance GROUND_SAFETY_BUFFER{0.015};

static constexpr Distance BIRD_RADIUS{0.062f};

static constexpr const TimePoint::duration SIMULATION_TIME_QUANTUM{75};

// TODO add pre-capture?
static constexpr std::chrono::milliseconds WEBCAM_POST_CAPTURE_PROCESSING_TIME = 12ms; // guessed
// this is how long a call to XGetImage() takes, let's assume the state of the screen is captured immediately
// and all the time is spent in postprocessing/data transfer
static constexpr std::chrono::milliseconds SCREEN_POST_CAPTURE_PROCESSING_TIME = 20ms;

// ----------- arm control ------------

// time between tap() is issued to the arm and the game registering the tap
// also - wait this long after issuing a tap() to the arm before lifting
// going much lower than this causes the arm to be lifted too quickly and the tablet doesn't register the tap
static constexpr std::chrono::milliseconds PHYSICAL_ARM_TAP_DELAY = 70ms;
static constexpr std::chrono::milliseconds SIMULATED_ARM_TAP_DELAY = 30ms;

// wait at least this long before issuing the next tap()
// could probably be lower but we're getting too many taps right now, so limiting spam
static constexpr std::chrono::milliseconds PHYSICAL_ARM_LIFT_DELAY = 50ms;
static constexpr std::chrono::milliseconds SIMULATED_ARM_LIFT_DELAY = 30ms;
